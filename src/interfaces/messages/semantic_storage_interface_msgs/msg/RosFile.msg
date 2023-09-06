# This file representation is used to exchange binary data.
# As the endianess isn't stored, only files with a byte order mark (BOM) or
# an implicitly specified endianess should be transferred.
string name   # file name
uint8[] data   # binary data 
