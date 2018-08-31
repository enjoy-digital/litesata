from litex.soc.tools.remote import RemoteClient

wb = RemoteClient()
wb.open()

# # #

# get identifier
fpga_id = ""
for i in range(256):
    c = chr(wb.read(wb.bases.identifier_mem + 4*i) & 0xff)
    fpga_id += c
    if c == "\0":
        break
print("fpga_id: " + fpga_id)

# get frequency
print("frequency : {}MHz".format(wb.constants.system_clock_frequency/1000000))

# # #

wb.close()
