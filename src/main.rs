//! This example runs on the Raspberry Pi Pico with a Waveshare board containing a Semtech Sx1262 radio.
//! It demonstrates LoRaWAN join functionality.

#![no_std]
#![no_main]

mod gps;

use cayenne_lpp::{CayenneLPP, LPP_GPS_SIZE};
use cayenne_lpp::{LPP_ANALOG_INPUT_SIZE, LPP_TEMPERATURE_SIZE};
use const_hex::decode_to_array;
use core::fmt::Error;
use core::sync::atomic::AtomicU16;
use core::sync::atomic::Ordering::Relaxed;
use core::{u16, usize};
use cortex_m::asm::delay;
use cyw43::Control;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Async, Channel};
use embassy_rp::pio::{Pio};
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::peripherals::{DMA_CH2, PIN_23, PIN_25, UART0};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::spi::Spi;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartRx};
use embassy_rp::{bind_interrupts, uart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, TimeoutError, Timer, with_timeout};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_io_async::Read;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::lorawan_radio::LorawanRadio;
use lora_phy::sx126x::{self, Sx126x, Sx126xVariant, TcxoCtrlVoltage};
use lora_phy::LoRa;
use lorawan_device::async_device::SendResponse;
use lorawan_device::async_device::{
    radio, region, Device, EmbassyTimer, JoinMode, JoinResponse, Timings,
};
use lorawan_device::default_crypto::DefaultFactory as Crypto;
use lorawan_device::region::DR;
use lorawan_device::{AppEui, AppKey, CryptoFactory, DevEui, RngCore};
use nmea::Nmea;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use {defmt_rtt as _, panic_probe as _};

// warning: set these appropriately for the region
const LORAWAN_REGION: region::Region = region::Region::EU868;
const MAX_TX_POWER: u8 = 14;

// warning: change these if using another device
// These are in the order that can be pasted into chirpstack/ttn, the EUIs will be reversed (to LSB)
// suiince this is what the rust code expects
const DEV_EUI: &str = "5945b83dc486feb5";
const APP_EUI: &str = "6b8dc8c69a4d619c";
const APP_KEY: &str = "ce90f0e5300eb44cf5bb4b3106d68bf4";

// This is the Timeout for the GPS – how long we should wait for it to obtain a position
// before sending a message without a position
const GPS_TIMEOUT: Duration = Duration::from_secs(2); 

bind_interrupts!(struct Irqs {
	ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
	UART0_IRQ => BufferedInterruptHandler<UART0>;
	PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});
static STATUS_BLINK_MILLIS: Signal<ThreadModeRawMutex, u16> = Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    //----------------Initialize the WiFi hardware-------------
    // Only done in order to get a blinking LED; slighlty overkill…
    {
        let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
        let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

        let pwr = Output::new(p.PIN_23, Level::Low);
        let cs = Output::new(p.PIN_25, Level::High);
        let mut pio = Pio::new(p.PIO0, Irqs);
        let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH2);

        static STATE: StaticCell<cyw43::State> = StaticCell::new();
        let state = STATE.init(cyw43::State::new());
        let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
        unwrap!(spawner.spawn(wifi_task(runner)));

        control.init(clm).await;
        control
            .set_power_management(cyw43::PowerManagementMode::SuperSave)
            .await;

        unwrap!(spawner.spawn(blink_with_frequency(control)));
    };

    //----------------Initialize the LoRa Radio-----------------
    let mut device = {
        let nss = Output::new(p.PIN_3.degrade(), Level::High);
        let reset = Output::new(p.PIN_15.degrade(), Level::High);
        let dio1 = Input::new(p.PIN_20.degrade(), Pull::None);
        let busy = Input::new(p.PIN_2.degrade(), Pull::None);

        let spi = Spi::new(
            p.SPI1,
            p.PIN_10,
            p.PIN_11,
            p.PIN_12,
            p.DMA_CH0,
            p.DMA_CH1,
            embassy_rp::spi::Config::default(),
        );
        let spi = ExclusiveDevice::new(spi, nss, Delay);
        let config = sx126x::Config {
            chip: Sx126xVariant::Sx1262,
            tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
            use_dcdc: true,
            use_dio2_as_rfswitch: true,
            rx_boost: false,
        };
        let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();
        let lora = LoRa::new(Sx126x::new(spi, iv, config), true, Delay)
            .await
            .unwrap();

        let radio: LorawanRadio<_, _, MAX_TX_POWER> = lora.into();
        let region: region::Configuration = region::Configuration::new(LORAWAN_REGION);
        let mut device: Device<_, Crypto, _, _> = Device::new(
            region,
            radio,
            EmbassyTimer::new(),
            embassy_rp::clocks::RoscRng,
        );
        // This joins the device to the LoRa network. The first join may fail, then the method backs off and tries again
        join_network(&mut device).await;
        device
    };

    //---------------------Initialize the ADC to read temperature and battery voltage-------------
    let mut adc = Adc::new(p.ADC, Irqs, embassy_rp::adc::Config::default());
    let mut bat_chan = Channel::new_pin(p.PIN_26, Pull::None);
    let mut temp_chan = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    //---------------------Initialize the GPS UART----------------------
    let mut rx = {
        let mut config = uart::Config::default();
        config.baudrate = 9600;
        let (tx_pin, rx_pin, uart) = (p.PIN_0, p.PIN_1, p.UART0);
        const UART_BUFFER_SIZE: usize = 16; // In practice, we only get 4 messages between read calls
        static TX_BUF: StaticCell<[u8; UART_BUFFER_SIZE]> = StaticCell::new();
        let tx_buf = &mut TX_BUF.init([0; UART_BUFFER_SIZE])[..];
        static RX_BUF: StaticCell<[u8; UART_BUFFER_SIZE]> = StaticCell::new();
        let rx_buf = &mut RX_BUF.init([0; UART_BUFFER_SIZE])[..];
        let uart = BufferedUart::new(uart, Irqs, tx_pin, rx_pin, tx_buf, rx_buf, config);
        let (mut rx, _) = uart.split();
        rx
    };


    //-------------------Main Loop---------------------

    //Debug: Set high data rate and low delay
    device.set_datarate(DR::_5);
    let mut delay_millis = 10000;
    let acknowledge = false;
    //let delay_millis = 30 * 1000;
    //let acknowledge = false;

    let mut buffer: [u8; LPP_TEMPERATURE_SIZE
        + LPP_ANALOG_INPUT_SIZE
        + LPP_ANALOG_INPUT_SIZE
        + LPP_GPS_SIZE] =
        [0; LPP_TEMPERATURE_SIZE + LPP_ANALOG_INPUT_SIZE + LPP_ANALOG_INPUT_SIZE + LPP_GPS_SIZE];
    let mut lpp = CayenneLPP::new(&mut buffer);

    loop {
        //---------------------------------Acquire Sensor Data-------------------------------------
        STATUS_BLINK_MILLIS.signal(500);
        let (battery_voltage, temperature) =
            battery_voltage_and_temperature(&mut bat_chan, &mut temp_chan, &mut adc).await;

        let gps_result = with_timeout(GPS_TIMEOUT, read_gps_data(&mut rx)).await;
        match gps_result {
            Ok((latitude, longitude, altitude, hdop)) => {
                lpp.add_analog_input(1, hdop).unwrap();
                lpp.add_gps(0, latitude as f32, longitude as f32, altitude)
                    .unwrap();
            }
            Err(_) => ()
        };

        lpp.add_analog_input(0, battery_voltage).unwrap();
        lpp.add_temperature(0, temperature).unwrap();
        
        let cayenne_lpp_payload = lpp.payload_slice();

        STATUS_BLINK_MILLIS.signal(50);
        let resp = device.send(cayenne_lpp_payload, 1, acknowledge).await;
        match resp {
            Ok(send_resp) => {
                info!("Sending okay: {:?}", send_resp);
                match send_resp {
                    SendResponse::DownlinkReceived(_) => {
                        // We have received a donlink, but it does not necessarily contain information
                        let downlink = device.take_downlink();
                        match downlink {
                            None => info!("Downlink empty!"),
                            Some(data) => match data.fport {
                                1 => {
                                    if data.data.len() == 1 && data.data[0] <= 6 {
                                        info!("FPORT 1: Setting Data rate to {}", data.data[0]);
                                        device.set_datarate(dr_from_u8(data.data[0]).unwrap());
                                    } else {
                                        error!("FPORT 1 But invalid data (or length)");
                                    }
                                }
                                2 => {
                                    if data.data.len() == 2 {
                                        let new_delay_millis: u16 =
                                            (data.data[0] as u16) * 256 + data.data[1] as u16;
                                        info!(
											"Changing delay between messages to {}",
											new_delay_millis
										);
                                        delay_millis = new_delay_millis as u64;
                                    } else {
                                        error!("FPORT 2 But invalid data length…");
                                    }
                                }
                                _ => error!("Invalid FPORT!"),
                            },
                        }
                    }
                    // If our session expired, we try to rejoin. We set the radio to the lowest data rate first.
                    SendResponse::SessionExpired => {
                        device.set_datarate(DR::_0);
                        join_network(&mut device).await;
                    }
                    SendResponse::NoAck => info!("No Acknowledgement received."),
                    SendResponse::RxComplete => info!("No data received."),
                }
            }
            Err(e) => warn!("Unexpected error: {:?}", e),
        }
        lpp.reset();

        STATUS_BLINK_MILLIS.signal(2000);
        Timer::after_millis(delay_millis).await;
    }
}

/// Attempt to join the LoRa network, with an exponential backoff in case of join failure
async fn join_network<R, C, T, G>(device: &mut Device<R, C, T, G>)
    where
        R: radio::PhyRxTx + Timings,
        T: radio::Timer,
        C: CryptoFactory + Default,
        G: RngCore,
{
    let mut join_attempt_count = 0;
    loop {
        info!(
			"Joining LoRaWAN network, attempt {:?}",
			join_attempt_count + 1
		);
        // The DEV_EUI and APP_EUI need to be reversed before putting them unto the device, since the default byte order differs
        // The key does not need that, for some reason.
        let mut dev_eui = decode_to_array(DEV_EUI).unwrap();
        dev_eui.reverse();
        let mut app_eui = decode_to_array(APP_EUI).unwrap();
        app_eui.reverse();
        let resp = device
            .join(&JoinMode::OTAA {
                deveui: DevEui::from(dev_eui),
                appeui: AppEui::from(app_eui),
                appkey: AppKey::from(decode_to_array(APP_KEY).unwrap()),
            })
            .await;

        let join_success = match resp {
            Ok(resp) => match resp {
                JoinResponse::JoinSuccess => {
                    info!("LoRa join request successfully accepted.");
                    true
                }
                JoinResponse::NoJoinAccept => {
                    info!("LoRa join request not acknowledged.");
                    false
                }
            },
            Err(e) => {
                warn!("LoRa join request failed with unknown error!: {:?}", e);
                false
            }
        };

        if join_success {
            break;
        }
        //Exponential backoff, up to 2048 seconds
        // Start at 1, then 2, then 4 …
        Timer::after_secs(2_u64.pow(join_attempt_count)).await;
        if join_attempt_count < 11 {
            join_attempt_count += 1
        }
    }
}


#[embassy_executor::task]
async fn wifi_task(runner: cyw43::Runner<'static, Output<'static, PIN_23>, PioSpi<'static, PIN_25, PIO0, 0, DMA_CH2>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn blink_with_frequency(mut control: Control<'static>) -> ! {
    // Initialize the blinking frequency to 500ms
    let mut blinking_delay: u16 = 100;
    let mut current_state = true;
    loop {
        // Toggle the LED, then wither wait for the current frequencies timeout
        // (continuining blinking with the same frequency) or update the frequency
        // right away by taking the signal's value as new frequency
        control.gpio_set(0, current_state).await;
        current_state = !current_state;
        let wait_result = with_timeout(Duration::from_millis(blinking_delay as u64), STATUS_BLINK_MILLIS.wait()).await;
        match wait_result { 
            Ok(new_value) => blinking_delay = new_value,
            Err(_) => (),
        }
    }
}


async fn battery_voltage_and_temperature(
    bat_chan: &mut Channel<'static>,
    temp_chan: &mut Channel<'static>,
    adc: &mut Adc<'static, Async>,
) -> (f32, f32) {
    const SAMPLE_COUNT: usize = 50;

    let mut battery_results: [u16; SAMPLE_COUNT] = [0; SAMPLE_COUNT];
    for battery_result in battery_results.iter_mut() {
        *battery_result = adc.read(bat_chan).await.unwrap();
        //Sampling delay
        Timer::after_millis(100).await;
    }
    let battery_result = median(&mut battery_results);
    let battery_voltage = 3.0 * battery_result as f32 * 3.3 / 4096.0;
    info!("battery_voltage: {:?}", battery_voltage);

    let mut temperature_results: [u16; SAMPLE_COUNT] = [0; SAMPLE_COUNT];
    for temperature_result in temperature_results.iter_mut() {
        *temperature_result = adc.read(temp_chan).await.unwrap();
        //Sampling delay
        Timer::after_millis(100).await;
    }
    let temperature_result = median(&mut temperature_results);
    let temperature = convert_to_celsius(temperature_result);
    info!("temperature: {:?}", temperature);

    (battery_voltage, temperature)
}

pub enum NmeaState {
    NoMessage,
    MessageStarted,
    MessageFinished,
}

async fn read_gps_data(rx: &mut BufferedUartRx<'static, UART0>) -> (f64, f64, f32, f32) {
    const SENTENCE_LENGTH: usize = 82;
    let mut nmea = Nmea::default();
    let mut nmea_state = NmeaState::NoMessage;
    let mut nmea_sentence_buf: [u8; SENTENCE_LENGTH] = [0; SENTENCE_LENGTH];
    let mut nmea_sentence_buf_idx: usize = 0;
    loop {
        let mut buf = [0; 16];
        let read_result = rx.read(&mut buf).await;
        match read_result {
            Ok(read_count) => {
                trace!("RX {:?}", buf);

                // If we do are not receiving a current message yet, wait for the '$' char
                match nmea_state {
                    NmeaState::MessageStarted => {
                        for buf_idx in 0..read_count {
                            let byte_read = buf[buf_idx];

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
                            let byte_read = buf[buf_idx];
                            if !valid_start {
                                let start_byte = '$' as u8;
                                if byte_read == start_byte {
                                    valid_start = true;

                                    nmea_state = NmeaState::MessageStarted;
                                    // Reset the sentence buffer
                                    nmea_sentence_buf = [0; SENTENCE_LENGTH];
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
                        // The message is finished, parse it
                        let sentence = core::str::from_utf8(&nmea_sentence_buf).unwrap();
                        info!("sentence {:?}", sentence);
                        let result = nmea.parse(sentence);
                        match result {
                            Ok(_) => info!("Sentence Received!"),
                            Err(_) => info!("Error!"),
                        }
                        nmea_state = NmeaState::NoMessage;
                    }
                }
            }

            Err(e) => info!("Recv Error: {:?}", e),
        }

        // Check if the NMEA data is complete
        if nmea.latitude.is_some()
            && nmea.longitude.is_some()
            && nmea.hdop.is_some()
            && nmea.altitude.is_some()
        {
            return (
                nmea.latitude.unwrap(),
                nmea.longitude.unwrap(),
                nmea.altitude.unwrap(),
                nmea.hdop.unwrap(),
            );
        }
    }
}

/// Calcualtes the median by sorting the array and taking the middle value
fn median<T>(buf: &mut [T]) -> T
    where
        T: Ord + Copy,
{
    // We seriously need to implement a sorting algorithm here.
    for i in 0..buf.len() {
        let mut j = i;
        while j > 0 && buf[j - 1] > buf[j] {
            buf.swap(j - 1, j);
            j -= 1;
        }
    }

    let index_of_middle = (buf.len() - 1) / 2;
    buf[index_of_middle]
}

fn convert_to_celsius(raw_temp: u16) -> f32 {
    // According to chapter 4.9.5. Temperature Sensor in RP2040 datasheet
    let temp = 27.0 - (raw_temp as f32 * 3.3 / 4096.0 - 0.706) / 0.001721;
    let sign = if temp < 0.0 { -1.0 } else { 1.0 };
    let rounded_temp_x10: i16 = ((temp * 10.0) + 0.5 * sign) as i16;
    (rounded_temp_x10 as f32) / 10.0
}

fn dr_from_u8(value: u8) -> Result<DR, Error> {
    match value {
        0 => Ok(DR::_0),
        1 => Ok(DR::_1),
        2 => Ok(DR::_2),
        3 => Ok(DR::_3),
        4 => Ok(DR::_4),
        5 => Ok(DR::_5),
        6 => Ok(DR::_6),
        _ => Err(Error),
    }
}
