extern crate serial;
extern crate uart_bno055;

use serial::{BaudRate, CharSize, FlowControl, Parity, PortSettings, SerialPort, StopBits};
use uart_bno055::*;

fn main() {
    let mut serial = serial::open("/dev/ttyAMA0").unwrap();
    serial
        .configure(&PortSettings {
            baud_rate: BaudRate::Baud115200,
            char_size: CharSize::Bits8,
            parity: Parity::ParityNone,
            stop_bits: StopBits::Stop1,
            flow_control: FlowControl::FlowNone,
        })
        .unwrap();
    serial.set_timeout(std::time::Duration::from_secs(5));
    let mut bno = BNO055::new(serial).unwrap();
    println!("{:?}", bno.get_revision().unwrap());
    bno.set_mode(BNO055OperationMode::NdofFmcOff).unwrap();
    bno.set_page(BNO055RegisterPage::Page0).unwrap();
    loop {
        let calib = bno.get_calibration_status().unwrap();
        eprintln!(
                    "S:{} G:{} A:{} M:{}",
                    calib.sys,
                    calib.gyr,
                    calib.acc,
                    calib.mag,
                );

        let quat = bno.get_quaternion().unwrap();
        let accel = bno.get_linear_acceleration().unwrap();
        println!(
            "{} {} {} {} {} {} {}",
            quat.w,
            quat.x,
            quat.y,
            quat.z,
            accel.0,
            accel.1,
            accel.2
        );

        std::thread::sleep_ms(10);
    }
}
