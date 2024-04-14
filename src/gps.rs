use defmt::{info, trace, warn};
use embassy_rp::{Peripheral, uart};
use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUartRx, Instance, RxPin,
};
use embassy_time::Instant;
use embedded_io_async::Read;
use nmea::Nmea;
use static_cell::StaticCell;

const UART_BUFFER_SIZE: usize = 8; // In practice, we only get 4 bytes between read calls
const NMEA_SENTENCE_LENGTH: usize = 82;

#[derive(Copy, Clone)]
pub struct PositionData {
    pub time_to_fix_ms: u32,
    pub hdop: f32,
    pub num_of_fix_satellites: u32,

    pub latitude: f32,
    pub longitude: f32,
    pub altitude: f32,
}

pub struct Gps<'d, T: Instance, PO: Pin> {
    pub enable_pin: Output<'d, PO>,
    uart: BufferedUartRx<'d, T>,
}

impl<'d, T: Instance, PO: Pin> Gps<'d, T, PO> {
    /// Sets up the GPS UART
    fn initialize_uart(
        uart: impl Peripheral<P = T> + 'd,
        irq: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>,
        rx: impl Peripheral<P = impl RxPin<T>> + 'd,
    ) -> BufferedUartRx<'d, T> {
        let mut config = uart::Config::default();
        config.baudrate = 9600;

        static RX_BUF: StaticCell<[u8; UART_BUFFER_SIZE]> = StaticCell::new();
        let rx_buf = &mut RX_BUF.init([0; UART_BUFFER_SIZE])[..];

        BufferedUartRx::new(uart, irq, rx, rx_buf, config)
    }
    pub fn new(
        uart: impl Peripheral<P = T> + 'd,
        irq: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>,
        rx: impl Peripheral<P = impl RxPin<T>> + 'd,
        enable_pin: PO,
    ) -> Self {
        let uart = Self::initialize_uart(uart, irq, rx);

        //Initialize the enable pin to Low = off. High turns the GPS on
        let enable_pin = Output::new(enable_pin, Level::Low);
        Self {
            enable_pin: enable_pin,
            uart: uart,
        }
    }

    /// Uses the UART to synchronize on the start of the sentence and read in a complete sentence
    async fn read_nmea_sentence(&mut self, nmea_sentence_buf: &mut [u8; NMEA_SENTENCE_LENGTH]) -> () {
        enum NmeaState {
            NoMessage,
            MessageStarted,
            MessageFinished,
        }

        let mut serial_input_buf = [0; UART_BUFFER_SIZE];
        let mut nmea_sentence_buf_idx: usize = 0;
        let mut nmea_state = NmeaState::NoMessage;
        loop {
            let read_result = self.uart.read(&mut serial_input_buf).await;
            match read_result {
                Ok(read_count) => {
                    trace!("RX {:?}", serial_input_buf);

                    // If we do are not receiving a current message yet, wait for the '$' char
                    match nmea_state {
                        NmeaState::MessageStarted => {
                            for buf_idx in 0..read_count {
                                let byte_read = serial_input_buf[buf_idx];

                                // Put the byte into the buffer, if we're not overflowing
                                if nmea_sentence_buf_idx < nmea_sentence_buf.len() {
                                    nmea_sentence_buf[nmea_sentence_buf_idx] = byte_read;
                                    nmea_sentence_buf_idx += 1;
                                } else {
                                    nmea_state = NmeaState::NoMessage;
                                }

                                let end_byte = '\n' as u8;
                                if byte_read == end_byte {
                                    nmea_state = NmeaState::MessageFinished
                                }
                            }
                        }
                        NmeaState::NoMessage => {
                            let mut valid_start = false;
                            // Check the rx buffer for a start byte
                            for buf_idx in 0..read_count {
                                let byte_read = serial_input_buf[buf_idx];
                                if !valid_start {
                                    let start_byte = '$' as u8;
                                    if byte_read == start_byte {
                                        valid_start = true;

                                        nmea_state = NmeaState::MessageStarted;
                                        // Reset the sentence buffer
                                        *nmea_sentence_buf = [0; NMEA_SENTENCE_LENGTH];
                                        nmea_sentence_buf_idx = 0;
                                    }
                                }
                                if valid_start {
                                    // We are receiving parts of a message, dump them to the bffer
                                    nmea_sentence_buf[nmea_sentence_buf_idx] = byte_read;
                                    nmea_sentence_buf_idx += 1;
                                }
                            }
                        }
                        NmeaState::MessageFinished => {
                            // The message is finished, return so the parser can do its work
                            return
                        }
                    }
                }

                Err(_) => warn!("UART Read error encountered!"),
            }
        }

    }

    pub async fn get_postition(&mut self) -> PositionData {
        let mut nmea_sentence_buf: [u8; NMEA_SENTENCE_LENGTH] = [0; NMEA_SENTENCE_LENGTH];
        let mut nmea = Nmea::default();

        // Remeber the current time, in order to tell how long we took to qcquire the position
        self.enable_pin.set_high();
        let start_time = Instant::now();

        loop {
            // Read from the serial port until we have a complete sentence in the buffer
            self.read_nmea_sentence(&mut nmea_sentence_buf).await;

            // Turn it into a string and update the parser
            let sentence = core::str::from_utf8(&nmea_sentence_buf).unwrap();
            info!("sentence {:?}", sentence);
            let result = nmea.parse(sentence);
            match result {
                Ok(_) => info!("Sentence Received!"),
                Err(_) => warn!("Error!"),
            }

            // Check if we have all required fields in order to return a PositionData struct
            if nmea.num_of_fix_satellites.is_some()
            && nmea.hdop.is_some() && nmea.latitude.is_some()
                && nmea.longitude.is_some()
                && nmea.altitude.is_some()
            {
                self.enable_pin.set_low();
                let fix_time = Instant::now();
                let time_to_fix = (fix_time - start_time).as_millis() as u32;
                
                return PositionData {
                    time_to_fix_ms: time_to_fix,
                    hdop: nmea.hdop.unwrap(),
                    num_of_fix_satellites: nmea.num_of_fix_satellites.unwrap(),
                    
                    latitude: nmea.latitude.unwrap() as f32,
                    longitude:nmea.longitude.unwrap() as f32,
                    altitude: nmea.altitude.unwrap(),
                }
            }
        }
    }
}
