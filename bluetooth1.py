import bluetooth


target_name = "helloESP"
target_address = None

nearby_devices = devices = bluetooth.discover_devices(duration=8, lookup_names = True)
print(nearby_devices)
for bdaddr in nearby_devices:
    print(bdaddr)
    if target_name == bluetooth.lookup_name( bdaddr ):
        target_address = bdaddr
        break

if target_address is not None:
    print("hlloo")
    print ("found target bluetooth device with address ", target_address[1])
else:
    print ("could not find target bluetooth device nearby")
