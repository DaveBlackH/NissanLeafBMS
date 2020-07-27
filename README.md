//Nissan Leaf BMS 30KWH data request and read via CANBUS
//
//Written by David Blackhurst 27/7/2020
//
//Good to use on a personal basis, not for commercial use unless preagreed with myself
//
//Lots of time has been spent on this to make it as easy to use as possible, I use the Arduino with Can Bus shield from the below link
//
//https://www.hobbytronics.co.uk/leonardo-canbus?keyword=canbus
//
//If you feel like contributing to my ongoing Electric Car Conversions and code writing please do,
//
//https://www.paypal.me/DBlackhurst
//
//This code will query the BMS over the EV CanBus, it will collect...
//Cell Voltages, Cell Shunt Activity, SOC, SOH, HV Voltage, LV Voltage, Capacity, Temperatures (3), Current
//
//This code is without warranty, it is untested on all Nissan Leaf variations and I cannot be held responsible for any damage caused by this code
//
//USE AT YOUR OWN RISK
