
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

def Hexcon(intval):
    if intval < 0:
        hexval = str(format((intval + (1 << 16)) % (1 << 16),'x'))
        h = hexval[2:4]+hexval[0:2]
        return h
    else:
        hexval = str(hex(intval))
        hexbit = 6-len(hexval)
        if len(hexval)<6:
            h = hexbit*"0"+hexval[2:]
            h = h[2:4]+h[0:2]
        else:
            h =hexval
            h = h[2:4]+h[0:2]
        return h