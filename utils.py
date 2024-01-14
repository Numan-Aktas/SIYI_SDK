
def toHex(intval, nbits):
    h = format((intval + (1 << nbits)) % (1 << nbits),'x')
    if len(h)==1:
        h="0"+h
    return h

def toInt(hexval):
    bits = 16
    val = int(hexval, bits)
    if val & (1 << (bits-1)):
        val -= 1 << bits
    return val

