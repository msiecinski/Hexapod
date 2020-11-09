#include <Ps4control.hpp>
#include <Movement.hpp>
#include "USBHost_t36.h"

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
USBHIDParser hid4(myusb);
USBHIDParser hid5(myusb);
JoystickController joystick1(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device

RawHIDController rawhid1(myusb);
RawHIDController rawhid2(myusb, 0xffc90004);

USBDriver *drivers[] = {&hub1, &hub2, &joystick1, &bluet, &hid1, &hid2, &hid3, &hid4, &hid5};

#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1", "Hub2", "JOY1D", "Bluet", "HID1" , "HID2", "HID3", "HID4", "HID5"};

bool driver_active[CNT_DEVICES] = {false, false, false, false};

// Lets also look at HID Input devices
USBHIDInput *hiddrivers[] = {&joystick1, &rawhid1, &rawhid2};

#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_DEVICES] = {"Joystick1", "RawHid1", "RawHid2"};

bool hid_driver_active[CNT_DEVICES] = {false, false, false};

BTHIDInput *bthiddrivers[] = {&joystick1};
#define CNT_BTHIDDEVICES (sizeof(bthiddrivers)/sizeof(bthiddrivers[0]))
const char * bthid_driver_names[CNT_HIDDEVICES] = {"joystick"};
bool bthid_driver_active[CNT_HIDDEVICES] = {false};

uint64_t joystick_full_notify_mask = (uint64_t) - 1;
int psAxis[64];
uint32_t buttons;


void SetupPS4()
{
   myusb.begin();
}

void LoopPS4()
{   
    uint8_t analogButton,digitalButton;

    myusb.Task();
    joystick1.axisChangeNotifyMask(joystick_full_notify_mask);
 
    if (joystick1.available())
    {    
        analogButton = 0;
        digitalButton = 0x8;
        joystick1.axisChangeNotifyMask(0xFFFFFl);
         for (uint8_t i = 0; i < 64; i++) 
        {
            psAxis[i] = joystick1.getAxis(i);
        }
        if(psAxis[1]>200)
            analogButton |= 0x1;
        if(psAxis[1]<60)
            analogButton |= 0x4;
        if(psAxis[5]<60)
            digitalButton |= 0x1;
        joystick1.joystickType();
        joystick1.joystickDataClear();
        Control(analogButton|(digitalButton<<8));
    }
}

