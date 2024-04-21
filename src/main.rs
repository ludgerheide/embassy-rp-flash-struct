//! This example runs on the Raspberry Pi Pico with a Waveshare board containing a Semtech Sx1262 radio.
//! It demonstrates LoRaWAN join functionality.

#![no_std]
#![no_main]

use core::fmt::Error;
use core::{u16, usize};

use const_hex::decode_to_array;
use cyw43::Control;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Async, Channel};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::peripherals::PIO0;
use embassy_rp::peripherals::{DMA_CH2, PIN_23, PIN_25, UART0};
use embassy_rp::pio::Pio;
use embassy_rp::spi::Spi;
use embassy_rp::uart::BufferedInterruptHandler;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use femtopb::{Message, Repeated, UnknownFields};
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
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};
use {defmt_rtt as _, panic_probe as _};

use crate::gps::Gps;
use crate::message::message::lora_message::StatusReport;
use crate::message::message::LoraMessage;

mod gps;
mod message;

const NONE_STATUS_REPORT: StatusReport = StatusReport {
    battery_voltage: None,
    temperature: None,
    time_to_first_fix: None,
    hdop: None,
    num_of_fix_satellites: None,
    latitude: None,
    longitude: None,
    altitude: None,
    unknown_fields: UnknownFields::empty(),
};
const NONE_OPTION_STATUS_REPORT: Option<StatusReport> = None;

// warning: set these appropriately for the region
const LORAWAN_REGION: region::Region = region::Region::EU868;
const MAX_TX_POWER: u8 = 14;

// warning: change these if using another device
// These are in the order that can be pasted into chirpstack/ttn, the EUIs will be reversed (to LSB)
// suiince this is what the rust code expects
const DEV_EUI: &str = include_str!("../device-config/DEV_EUI");
const APP_EUI: &str = include_str!("../device-config/APP_EUI");
const APP_KEY: &str = include_str!("../device-config/APP_KEY");

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
        let spi = PioSpi::new(
            &mut pio.common,
            pio.sm0,
            pio.irq0,
            cs,
            p.PIN_24,
            p.PIN_29,
            p.DMA_CH2,
        );

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

    //---------------------Initialize the ADC to read temperature and battery voltage-------------
    let mut adc = Adc::new(p.ADC, Irqs, embassy_rp::adc::Config::default());
    let mut bat_chan = Channel::new_pin(p.PIN_26, Pull::None);
    let mut temp_chan = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    //---------------------Initialize the GPS UART----------------------
    let mut gps = Gps::new(p.UART0, Irqs, p.PIN_1, p.PIN_4);

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

    //-------------------Main Loop---------------------

    // The initial data rate. Can be changed by sending an uplink with FPORT 1
    device.set_datarate(DR::_3);
    // How long to wait between messages. Can be set by sending an uplink with FPORT 2
    let mut interval_between_wakeups_ms: u32 = 60000;
    //How lonkg to wait for the GPS to aqcuire a position before giving up. Can be set by sending an uplink with FPORT 3
    let mut gps_timeout_ms: u32 = 240000;

    let mut sequence_counter: u32 = 0;
    let mut sequence_of_last_confirmation: Option<u32> = None;
    let mut previous_positions: [Option<StatusReport>; 3] = [NONE_OPTION_STATUS_REPORT; 3];

    let mut gps_did_ever_receive_valid_position = false;

    loop {
        //---------------------------------Acquire Sensor Data-------------------------------------
        STATUS_BLINK_MILLIS.signal(500);

        // Start the acquisition process for battery data (it runs in the background)
        let analog_data_future =
            battery_voltage_and_temperature(&mut bat_chan, &mut temp_chan, &mut adc);
        let gps_result = with_timeout(
            Duration::from_millis(gps_timeout_ms as u64),
            gps.get_postition(),
        )
        .await;
        let (battery_voltage, temperature) = analog_data_future.await;
        let current_report: StatusReport = match gps_result {
            Ok(pos) => {
                info!("Received GPS Position, packaging it…");
                gps_did_ever_receive_valid_position = false;
                StatusReport {
                    battery_voltage: Some(battery_voltage),
                    temperature: Some(temperature),
                    time_to_first_fix: Some(pos.time_to_fix_ms),
                    hdop: Some(pos.hdop),
                    num_of_fix_satellites: Some(pos.num_of_fix_satellites),
                    latitude: Some(pos.latitude),
                    longitude: Some(pos.longitude),
                    altitude: Some(pos.altitude),
                    unknown_fields: UnknownFields::empty(),
                }
            }
            Err(_) => {
                // In case of failure, send the GPS to sleep as well,
                if gps_did_ever_receive_valid_position {
                    gps.enable_pin.set_low();
                }
                info!("GPS Position timeout");
                StatusReport {
                    battery_voltage: Some(battery_voltage),
                    temperature: Some(temperature),
                    time_to_first_fix: None,
                    hdop: None,
                    num_of_fix_satellites: None,
                    latitude: None,
                    longitude: None,
                    altitude: None,
                    unknown_fields: UnknownFields::empty(),
                }
            }
        };

        //---------------------------Assemble the message----------------------------
        let mut unack_report_buf = [NONE_STATUS_REPORT; 3];
        let message = LoraMessage {
            sequence: Some(sequence_counter),
            sequence_of_last_confirmation: sequence_of_last_confirmation,
            current_report: Some(current_report),
            unacknowledged_reports: {
                match device.get_datarate() {
                    DR::_0 | DR::_1 | DR::_2 => Repeated::empty(),
                    DR::_3 => {
                        if previous_positions[0].is_some() {
                            unack_report_buf[0] = copy_sr(&previous_positions[0]);
                            Repeated::from_slice(&unack_report_buf[0..1])
                        } else {
                            Repeated::empty()
                        }
                    }
                    DR::_4 | DR::_5 | DR::_6 => {
                        // Send all previous positions we have. That may be 1,2, or 3
                        if previous_positions[0].is_some()
                            && previous_positions[1].is_some()
                            && previous_positions[2].is_some()
                        {
                            for i in 0..3 {
                                unack_report_buf[i] = copy_sr(&previous_positions[i]);
                            }
                            Repeated::from_slice(&unack_report_buf)
                        } else if previous_positions[0].is_some()
                            && previous_positions[1].is_some()
                            && previous_positions[2].is_some()
                        {
                            unack_report_buf[0] = copy_sr(&previous_positions[0]);
                            unack_report_buf[1] = copy_sr(&previous_positions[1]);
                            Repeated::from_slice(&unack_report_buf[0..2])
                        } else if previous_positions[0].is_some() {
                            unack_report_buf[0] = copy_sr(&previous_positions[0]);
                            Repeated::from_slice(&unack_report_buf[0..2])
                        } else {
                            Repeated::empty()
                        }
                    }
                    _ => Repeated::empty(),
                }
            },
            unknown_fields: UnknownFields::empty(),
        };

        let mut msg_buf: [u8; 256] = [0; 256];
        let encoded_len = message.encoded_len();
        let (fport, to_send) = {
            if encoded_len <= msg_buf.len() {
                let encode_result = message.encode(&mut msg_buf.as_mut_slice());
                match encode_result {
                    Ok(_) => (1 as u8, &msg_buf[0..encoded_len]),
                    Err(_) => (0 as u8, &msg_buf[0..0]),
                }
            } else {
                warn!("Encoded Length of message exceeds buffer size!");
                (0 as u8, &msg_buf[0..0])
            }
        };

        STATUS_BLINK_MILLIS.signal(50);
        let resp = device.send(to_send, fport, true).await;
        let mut should_add_position_to_previous = true;
        match resp {
            Ok(send_resp) => {
                info!("Sending okay: {:?}", send_resp);
                match send_resp {
                    SendResponse::DownlinkReceived(_) => {
                        // Discard all previous:positions (we have successully transmitteed them now
                        should_add_position_to_previous = false;
                        previous_positions = [NONE_OPTION_STATUS_REPORT; 3];
                        sequence_of_last_confirmation = Some(sequence_counter);

                        // Then Handle downlink requests
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
                                    if data.data.len() == 4 {
                                        let new_delay_millis: u32 = ((data.data[0] as u32) << 24)
                                            + ((data.data[1] as u32) << 16)
                                            + ((data.data[2] as u32) << 8)
                                            + ((data.data[3] as u32) << 0);
                                        info!(
                                            "Changing delay between messages to {}",
                                            new_delay_millis
                                        );
                                        interval_between_wakeups_ms = new_delay_millis;
                                    } else {
                                        error!("FPORT 2 But invalid data length…");
                                    }
                                }
                                3 => {
                                    if data.data.len() == 4 {
                                        let new_gps_timeout: u32 = ((data.data[0] as u32) << 24)
                                            + ((data.data[1] as u32) << 16)
                                            + ((data.data[2] as u32) << 8)
                                            + ((data.data[3] as u32) << 0);
                                        info!("Changing GPS Timeout to {}", new_gps_timeout);
                                        gps_timeout_ms = new_gps_timeout;
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
            Err(e) => warn!("Unexpected error! {:?}", e),
        }

        // If the unack_messages array is not empty at this point, we have not received a valid downlink.
        // In this case, shift its contents up one and then put the current value in it, discarding
        // Temperature and voltage
        if should_add_position_to_previous {
            if previous_positions[1].is_some() {
                previous_positions[2] = Some(copy_sr(&previous_positions[1]));
            }
            if previous_positions[0].is_some() {
                previous_positions[1] = Some(copy_sr(&previous_positions[0]));
            }
            previous_positions[0] = Some(copy_sr(&message.current_report));
        }
        sequence_counter += 1;

        STATUS_BLINK_MILLIS.signal(2000);
        Timer::after(Duration::from_millis(interval_between_wakeups_ms as u64)).await;
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
async fn wifi_task(
    runner: cyw43::Runner<
        'static,
        Output<'static, PIN_23>,
        PioSpi<'static, PIN_25, PIO0, 0, DMA_CH2>,
    >,
) -> ! {
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
        let wait_result = with_timeout(
            Duration::from_millis(blinking_delay as u64),
            STATUS_BLINK_MILLIS.wait(),
        )
        .await;
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
    const SAMPLE_COUNT: usize = 10;

    let mut battery_results: [u16; SAMPLE_COUNT] = [0; SAMPLE_COUNT];
    for battery_result in battery_results.iter_mut() {
        *battery_result = adc.read(bat_chan).await.unwrap();
        //Sampling delay
        Timer::after_millis(50).await;
    }
    let battery_result = median(&mut battery_results);
    let battery_voltage = 3.0 * battery_result as f32 * 3.3 / 4096.0;
    info!("battery_voltage: {:?}", battery_voltage);

    let mut temperature_results: [u16; SAMPLE_COUNT] = [0; SAMPLE_COUNT];
    for temperature_result in temperature_results.iter_mut() {
        *temperature_result = adc.read(temp_chan).await.unwrap();
        //Sampling delay
        Timer::after_millis(50).await;
    }
    let temperature_result = median(&mut temperature_results);
    let temperature = convert_to_celsius(temperature_result);
    info!("temperature: {:?}", temperature);

    (battery_voltage, temperature)
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

// Hacky copy() for status report
fn copy_sr<'a>(sr: &Option<StatusReport>) -> StatusReport<'a> {
    match sr {
        Some(sr) => StatusReport {
            battery_voltage: None,
            temperature: None,
            time_to_first_fix: sr.time_to_first_fix.clone(),
            hdop: sr.hdop.clone(),
            num_of_fix_satellites: sr.num_of_fix_satellites.clone(),
            latitude: sr.latitude.clone(),
            longitude: sr.longitude.clone(),
            altitude: sr.altitude.clone(),
            unknown_fields: UnknownFields::empty(),
            ..Default::default()
        },
        None => StatusReport::default(),
    }
}
