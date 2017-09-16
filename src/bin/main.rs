extern crate i2cdev_bno055;
extern crate i2csensors;
extern crate i2cdev;

use i2cdev_bno055::*;

#[cfg(any(target_os = "linux", target_os = "android"))]
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

#[cfg(not(any(target_os = "linux", target_os = "android")))]
fn main() {}

#[cfg(any(target_os = "linux", target_os = "android"))]
pub fn get_linux_bno055_i2c_device() -> Result<LinuxI2CDevice, LinuxI2CError> {
    match LinuxI2CDevice::new("/dev/i2c-1", BNO055_DEFAULT_ADDR) {
        Ok(device) => Ok(device),
        Err(e) => Err(e),
    }
}

#[cfg(any(target_os = "linux", target_os = "android"))]
fn main() {
    match get_linux_bno055_i2c_device() {
        Ok(device) => {
            let mut bno = BNO055::new(device).unwrap();
            println!("{:?}", bno.get_revision().unwrap());
            bno.set_mode(BNO055OperationMode::Ndof).unwrap();
            loop {
                let accel = bno.get_linear_acceleration().unwrap();
                println!("{:+2.2}\t{:+2.2}\t{:+2.2}", accel.x, accel.y, accel.z);
            }
        }
        Err(e) => {}
    }


}
