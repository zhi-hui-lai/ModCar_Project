#!/usr/bin/env python3
#=============================================================================

import os
import sys
import ctypes
import numpy as np

#=============================================================================

if(os.path.exists('/home/pi') == True):
    lib = ctypes.CDLL('/home/pi/eyebot/lib/dynamic/libeyebot.so')
    Sim = False
    
else:
    Sim = True
    print(sys.platform)
    if((sys.platform == 'linux') or (sys.platform == 'linux2')):
    	lib = ctypes.CDLL('libeyesim.so')
    
    elif((sys.platform == 'win32') or (sys.platform == 'cygwin')):
    	lib = ctypes.CDLL('cygeyesim.dll')
    	
    elif(sys.platform == 'darwin'):
    	lib = ctypes.CDLL('libeyesim.dylib')
    	
    else:
    	print('[Error!] EyeBot Library not Found')
    	sys.exit()

#=============================================================================

NOKEY  =  0
ANYKEY = -1
KEY1 = 1
KEY2 = 2
KEY3 = 4
KEY4 = 8
PSD_LEFT = 2
PSD_FRONT = 1
PSD_RIGHT = 3
PSD_BACK = 4
QQVGA = 0
QVGA = 1
VGA = 2
CAM1MP = 3
CAMHD = 4
CAM5MP = 5
CUSTOM = 10

QQVGA_X = 160
QQVGA_Y = 120
QQVGA_SIZE = QQVGA_X*QQVGA_Y*3
QQVGA_PIXELS = QQVGA_X*QQVGA_Y

QVGA_X = 320
QVGA_Y = 240
QVGA_SIZE = QVGA_X*QVGA_Y*3
QVGA_PIXELS = QVGA_X*QVGA_Y

VGA_X = 640
VGA_Y = 480
VGA_SIZE = VGA_X*VGA_Y*3
VGA_PIXELS = VGA_X*VGA_Y

CAM1MP_X = 1296
CAM1MP_Y = 730
CAM1MP_SIZE = CAM1MP_X*CAM1MP_Y*3
CAM1MP_PIXELS = CAM1MP_X*CAM1MP_Y

CAMHD_X = 1920
CAMHD_Y = 1080
CAMHD_SIZE = CAMHD_X*CAMHD_Y*3
CAMHD_PIXELS = CAMHD_X*CAMHD_Y

CAM5MP_X = 2592
CAM5MP_Y = 1944
CAM5MP_SIZE = CAM5MP_X*CAM5MP_Y*3
CAM5MP_PIXELS = CAM5MP_X*CAM5MP_Y

CAMWIDTH = int()
CAMHEIGHT = int()
CAMPIXELS = int()
CAMSIZE = int()

HELVETICA = 0
TIMES = 1
COURIER = 2

NORMAL = 0
BOLD = 1
ITALICS = 2

RED = 16711680
GREEN = 65280
BLUE = 255
WHITE = 16777215
GRAY = 8421504
BLACK = 0
SILVER = 12632256
LIGHTGRAY = 13882323
DARKGRAY = 11119017
NAVY = 128
CYAN = 61166
TEAL = 32896
MAGENTA = 16711935
PURPLE = 8388736
MAROON = 8388608
YELLOW = 16776960
OLIVE = 10145074
ORANGE = 16753920

CAMWIDTHARR = {0: QQVGA_X, 1: QVGA_X, 2: VGA_X, 3: CAM1MP_X, 4: CAMHD_X, 5: CAM5MP_X, 10: 0}
CAMHEIGHTARR = {0: QQVGA_Y, 1: QVGA_Y, 2: VGA_Y, 3: CAM1MP_Y, 4: CAMHD_Y, 5: CAM5MP_Y, 10: 0}
CAMPIXELSARR = {0: QQVGA_PIXELS, 1: QVGA_PIXELS, 2: VGA_PIXELS, 3: CAM1MP_PIXELS, 4: CAMHD_PIXELS, 5: CAM5MP_PIXELS, 10: 0}
CAMSIZEARR = {0: QQVGA_SIZE, 1: QVGA_SIZE, 2: VGA_SIZE, 3: CAM1MP_SIZE, 4: CAMHD_SIZE, 5: CAM5MP_SIZE, 10: 0}

#=============================================================================

lib.LCDPrintf.argtypes = [ctypes.c_char_p]
lib.LCDPrintf.restype = ctypes.c_int
lib.LCDSetPrintf.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_char_p]
lib.LCDSetPrintf.restype = ctypes.c_int
lib.LCDClear.argtypes = None
lib.LCDClear.restype = ctypes.c_int
lib.LCDSetPos.argtypes = [ctypes.c_int, ctypes.c_int]
lib.LCDSetPos.restype = ctypes.c_int
lib.LCDGetPos.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
lib.LCDGetPos.restype = ctypes.c_int
lib.LCDSetColor.argtypes = [ctypes.c_int, ctypes.c_int]
lib.LCDSetColor.restype = ctypes.c_int
lib.LCDSetFont.argtypes = [ctypes.c_int, ctypes.c_int]
lib.LCDSetFont.restype = ctypes.c_int
lib.LCDSetFontSize.argtypes = [ctypes.c_int, ctypes.c_int]
lib.LCDSetFontSize.restype = ctypes.c_int
lib.LCDSetMode.argtypes = [ctypes.c_int, ctypes.c_int]
lib.LCDSetMode.restype = ctypes.c_int
lib.LCDMenu.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p]
lib.LCDMenu.restype = ctypes.c_int
lib.LCDMenuI.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p]
lib.LCDMenuI.restype = ctypes.c_int
lib.LCDGetSize.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
lib.LCDGetSize.restype = ctypes.c_int
lib.LCDPixel.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.LCDPixel.restype = ctypes.c_int
lib.LCDGetPixel.argtypes = [ctypes.c_int, ctypes.c_int]
lib.LCDGetPixel.restype = ctypes.c_int
lib.LCDLine.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.LCDLine.restype = ctypes.c_int
lib.LCDArea.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.LCDArea.restype = ctypes.c_int
lib.LCDCircle.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.LCDCircle.restype = ctypes.c_int
lib.LCDImageSize.argtypes = [ctypes.c_int]
lib.LCDImageSize.restype = ctypes.c_int
lib.LCDImageStart.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.LCDImageStart.restype = ctypes.c_int
lib.LCDImage.argtypes = [ctypes.POINTER(ctypes.c_byte)]
lib.LCDImage.restype = ctypes.c_int
lib.LCDImageGray.argtypes = [ctypes.POINTER(ctypes.c_byte)]
lib.LCDImageGray.restype = ctypes.c_int
lib.LCDImageBinary.argtypes = [ctypes.POINTER(ctypes.c_byte)]
lib.LCDImageBinary.restype = ctypes.c_int
lib.LCDRefresh.argtypes = None
lib.LCDRefresh.restype = ctypes.c_int

def LCDPrintf(format, *args):
	final = (format % args)
	return lib.LCDPrintf(bytes(final.encode('ascii')))

def LCDSetPrintf(row, column, format, *args):
	final = (format % args)
	return lib.LCDSetPrintf(ctypes.c_int(row), ctypes.c_int(column), ctypes.c_char_p(final.encode('ascii')))

def LCDClear():
	return lib.LCDClear()

def LCDSetPos(row, column):
	return lib.LCDSetPos(ctypes.c_int(row), ctypes.c_int(column))

def LCDGetPos(row, column):
	return lib.LCDGetPos(ctypes.pointer(row), ctypes.pointer(column))

def LCDSetColor(fg, bg):
	return lib.LCDSetColor(ctypes.c_int(fg), ctypes.c_int(bg))

def LCDSetFont(font, variation):
	return lib.LCDSetFont(ctypes.c_int(font), ctypes.c_int(variation))

def LCDSetFontSize(fontsize):
	return lib.LCDSetFontSize(ctypes.c_int(fontsize))

def LCDSetMode(mode):
	return lib.LCDSetMode(ctypes.c_int(mode))

def LCDMenu(st1, st2, st3, st4):
	st1 = ctypes.c_char_p(st1.encode('ascii'))
	st2 = ctypes.c_char_p(st2.encode('ascii'))
	st3 = ctypes.c_char_p(st3.encode('ascii'))
	st4 = ctypes.c_char_p(st4.encode('ascii'))
	return lib.LCDMenu(st1, st2, st3, st4)

def LCDMenuI(pos, string, fg, bg):
	return lib.LCDMenu(ctypes.c_int(pos), ctypes.c_char_p(string.encode('ascii')), ctypes.c_int(fg), ctypes.c_int(bg))

def LCDGetSize(x, y):
	return lib.LCDGetSize(ctypes.pointer(x), ctypes.pointer(y))

def LCDPixel(x, y, col):
	return lib.LCDPixel(ctypes.c_int(x), ctypes.c_int(y), ctypes.c_int(col))

def LCDGetPixel(x, y):
	return lib.LCDGetPixel(ctypes.c_int(x), ctypes.c_int(y))

def LCDLine(x1, y1, x2, y2, col):
	return lib.LCDLine(ctypes.c_int(x1), ctypes.c_int(y1), ctypes.c_int(x2), ctypes.c_int(y2), ctypes.c_int(col))

def LCDArea(x1, y1, x2, y2, col, fill):
	return lib.LCDArea(ctypes.c_int(x1), ctypes.c_int(y1), ctypes.c_int(x2), ctypes.c_int(y2), ctypes.c_int(col), ctypes.c_int(fill))

def LCDCircle(x1, y1, size, col, fill):
	return lib.LCDCircle(ctypes.c_int(x1), ctypes.c_int(y1), ctypes.c_int(size), ctypes.c_int(col), ctypes.c_int(fill))

def LCDImageSize(t):
	return lib.LCDImageSize(ctypes.c_int(t))

def LCDImageStart(x, y, xs, ys):
	return lib.LCDImageStart(ctypes.c_int(x), ctypes.c_int(y), ctypes.c_int(xs), ctypes.c_int(ys))

def LCDImage(img):
	return lib.LCDImage(img)

def LCDImageGray(img_gray):
	return lib.LCDImageGray(img_gray)

def LCDImageBinary(binary):
	return lib.LCDImageGray(binary)

def LCDRefresh():
	return lib.LCDRefresh()

#=============================================================================

lib.KEYGet.argtypes = None
lib.KEYGet.restype = ctypes.c_int
lib.KEYRead.argtypes = None
lib.KEYRead.restype = ctypes.c_int
lib.KEYWait.argtypes = [ctypes.c_int]
lib.KEYWait.restype = ctypes.c_int
lib.KEYGetXY.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
lib.KEYGetXY.restype = ctypes.c_int
lib.KEYReadXY.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
lib.KEYReadXY.restype = ctypes.c_int

def KEYGet():
	return lib.KEYGet()

def KEYRead():
	return lib.KEYRead()

def KEYWait(key):
	return lib.KEYWait(ctypes.c_int(key))

def KEYGetXY(x, y):
	return lib.KEYGetXY(ctypes.pointer(x), ctypes.pointer(y))

def KEYReadXY(x, y):
	return lib.KEYReadXY(ctypes.pointer(x), ctypes.pointer(y))

#=============================================================================

lib.CAMInit.argtypes = [ctypes.c_int]
lib.CAMInit.restype = ctypes.c_int
lib.CAMRelease.argtypes = None
lib.CAMRelease.restype = ctypes.c_int
lib.CAMGet.argtypes = [ctypes.POINTER(ctypes.c_byte)]
lib.CAMGet.restype = ctypes.c_int
lib.CAMGetGray.argtypes = [ctypes.POINTER(ctypes.c_byte)]
lib.CAMGetGray.restype = ctypes.c_int

def CAMInit(resolution):
	global CAMSIZE
	global CAMPIXELS
	global CAMHEIGHT
	global CAMWIDTH
	CAMSIZE = CAMSIZEARR[resolution]
	CAMPIXELS = CAMPIXELSARR[resolution]
	CAMHEIGHT = CAMHEIGHTARR[resolution]
	CAMWIDTH = CAMWIDTHARR[resolution]
	return lib.CAMInit(ctypes.c_int(resolution))

def CAMRelease():
	lib.CAMRelease()

def CAMGet():
	ptr = (ctypes.c_byte * CAMSIZE)()
	lib.CAMGet(ctypes.cast(ptr, ctypes.POINTER(ctypes.c_byte)))
	return ptr

def CAMGetGray():
	ptr = (ctypes.c_byte * CAMPIXELS)()
	lib.CAMGetGray(ctypes.cast(ptr, ctypes.POINTER(ctypes.c_byte)))
	return ptr

#=============================================================================

lib.IPSetSize.argtypes = [ctypes.c_int]
lib.IPSetSize.restype = ctypes.c_int
lib.IPReadFile.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
lib.IPReadFile.restype = ctypes.c_int

lib.IPWriteFile.argtypes = [ctypes.c_char_p, ctypes.c_byte]
lib.IPWriteFile.restype = ctypes.c_int
lib.IPWriteFileGray.argtypes = [ctypes.c_char_p, ctypes.c_byte]
lib.IPWriteFileGray.restype = ctypes.c_int

lib.IPLaplace.argtypes = [ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte)]
lib.IPLaplace.restype =  None
lib.IPSobel.argtypes =   [ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte)]
lib.IPSobel.restype =    None

lib.IPCol2Gray.argtypes = [ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte)]
lib.IPCol2Gray.restype = None
lib.IPGray2Col.argtypes = [ctypes.c_byte, ctypes.c_byte]
lib.IPGray2Col.restype = None
lib.IPRGB2Col.argtypes = [ctypes.c_byte, ctypes.c_byte, ctypes.c_byte, ctypes.c_byte]
lib.IPRGB2Col.restype = None
lib.IPCol2HSI.argtypes = [ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte)]
lib.IPCol2HSI.restype = None
lib.IPOverlay.argtypes = [ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte)]
lib.IPOverlay.restype = None
lib.IPOverlayGray.argtypes = [ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte), ctypes.c_int, ctypes.POINTER(ctypes.c_byte)]
lib.IPOverlayGray.restype = None
lib.IPPRGB2Col.argtypes = [ctypes.c_byte, ctypes.c_byte, ctypes.c_byte]
lib.IPPRGB2Col.restype = ctypes.c_int
lib.IPPCol2RGB.argtypes = [ctypes.c_int, ctypes.c_byte, ctypes.c_byte, ctypes.c_byte]
lib.IPPCol2RGB.restype = None
lib.IPPCol2HSI.argtypes = [ctypes.c_int, ctypes.c_byte, ctypes.c_byte, ctypes.c_byte]
lib.IPPCol2HSI.restype = None
lib.IPPRGB2Hue.argtypes = [ctypes.c_byte, ctypes.c_byte, ctypes.c_byte]
lib.IPPRGB2Hue.restype = ctypes.c_byte
lib.IPPRGB2HSI.argtypes = [ctypes.c_byte, ctypes.c_byte, ctypes.c_byte, ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte)]
lib.IPPRGB2HSI.restype = None

def IPSetSize(resolution):
	return lib.IPSetSize(ctypes.c_int(resolution))

def IPReadFile(filename):
	f = open(filename, 'r')
	f.readline()
	header = f.readline()
	header = header.split(' ')
	len = int(header[0]) * int(header[1]) * 3
	img = (ctypes.c_char*len)()
	result = lib.IPReadFile(ctypes.c_char_p(filename.encode('ascii')), img)
	return [result, img]

def IPWriteFile(filename):
	return lib.IPWriteFile(ctypes.c_char_p(filename.encode('ascii')), img)

def IPWriteFileGray(filename, gray):
	return lib.IPWriteFile(ctypes.c_char_p(filename.encode('ascii')), gray)

def IPLaplace(grayIn):
	BUF = (ctypes.c_byte*CAMPIXELS)()
	result = lib.IPLaplace(grayIn, BUF)
	return BUF

def IPSobel(grayIn):
	BUF = (ctypes.c_byte*CAMPIXELS)()
	result = lib.IPSobel(grayIn, BUF)
	return BUF

def IPCol2Gray(imgIn):
	IPBUF = (ctypes.c_byte*CAMPIXELS)()
	result = lib.IPCol2Gray(imgIn, IPBUF)
	return IPBUF

def IPGray2Col(imgIn, colOut):
	lib.IPGray2Col(ctypes.c_byte(imgIn), ctypes.c_byte(colOut))

def IPRGB2Col(r, g, b, imgOut):
	lib.IPGray2Col(ctypes.c_byte(r), ctypes.c_byte(g), ctypes.pointer(ctypes.c_byte(imgOut)))

def IPCol2HSI(img):
	HBUF = (ctypes.c_byte*CAMPIXELS)()
	SBUF = (ctypes.c_byte*CAMPIXELS)()
	IBUF = (ctypes.c_byte*CAMPIXELS)()
	result = lib.IPCol2HSI(img, HBUF, SBUF, IBUF)
	return [HBUF, SBUF, IBUF]

def IPOverlay(c1, c2):
	lib.Overlay(ctypes.c_byte(c1), ctypes.c_byte(c2), ctypes.pointer(ctypes.c_byte(cOut)))
	BUF = (ctypes.c_byte*CAMPIXELS)()
	result = lib.IPLaplace(grayIn, BUF)
	return BUF

def IPOverlayGray(g1, g2, col):
	BUF = (ctypes.c_byte*CAMSIZE)()
	result = lib.IPOverlayGray(g1, g2, col, BUF)
	return BUF

def IPPRGB2Col(r, g, b):
	return lib.IPPRGB2Col(ctypes.c_byte(r), ctypes.c_byte(g), ctypes.c_byte(b))

def IPPCol2RGB(col, r, g, b):
	lib.IPPCol2RGB(ctypes.c_int(col), ctypes.pointer(ctypes.c_byte(r)), ctypes.pointer(ctypes.c_byte(g)), ctypes.pointer(ctypes.c_byte(b)))

def IPPCol2HSI(c, h, s, i):
	lib.IPPCol2HSI(ctypes.c_int(c), ctypes.pointer(ctypes.c_byte(h)), ctypes.pointer(ctypes.c_byte(s)), ctypes.pointer(ctypes.c_byte(i)))

def IPPRGB2Hue(r, g, b):
	return lib.IPPRGB2Hue(ctypes.c_byte(r), ctypes.c_byte(g), ctypes.c_byte(b))

def IPPRGB2HSI(r, g, b):
	h = ctypes.c_byte()
	s = ctypes.c_byte()
	i = ctypes.c_byte()
	lib.IPPRGB2HSI(ctypes.c_byte(r), ctypes.c_byte(g), ctypes.c_byte(b), h, s, i)
	return [h.value, s.value, i.value]

#=============================================================================

lib.OSExecute.argtypes = [ctypes.c_char_p]
lib.OSExecute.restype = ctypes.c_char_p
lib.OSVersion.argtypes = [ctypes.c_char_p]
lib.OSVersion.restype = ctypes.c_int
lib.OSVersionIO.argtypes = [ctypes.c_char_p]
lib.OSVersionIO.restype = ctypes.c_int
lib.OSMachineSpeed.argtypes = None
lib.OSMachineSpeed.restype = ctypes.c_int
lib.OSMachineType.argtypes = None
lib.OSMachineType.restype = ctypes.c_int
lib.OSMachineName.argtypes = [ctypes.c_char_p]
lib.OSMachineName.restype = ctypes.c_char_p
lib.OSMachineID.argtypes = None
lib.OSMachineID.restype = ctypes.c_int
lib.OSGetCount.argtypes = None
lib.OSGetCount.restype = ctypes.c_int

def OSExecute(command):
	return lib.OSExecute(ctypes.c_char_p(command.encode('ascii')))

def OSVersion(buf):
	return lib.OSVersion(ctypes.c_char_p(buf.encode('ascii')))

def OSVersionIO(buf):
	return lib.OSVersionIO(ctypes.c_char_p(buf.encode('ascii')))

def OSMachineSpeed():
	return lib.OSMachineSpeed(None)

def OSMachineType():
	return lib.OSMachineType(None)

def OSMachineName(buf):
	return lib.OSMachineName(ctypes.c_char_p(buf.encode('ascii')))

def OSMachineID():
	return lib.OSMachineID(None)

#=============================================================================

lib.OSWait.argtypes = [ctypes.c_int]
lib.OSWait.restype = ctypes.c_int
lib.OSAttachTimer.argtypes = [ctypes.c_int, ctypes.c_void_p]
lib.OSAttachTimer.restype = ctypes.c_int
lib.OSDetachTimer.argtypes = [ctypes.c_int]
lib.OSDetachTimer.restype = ctypes.c_int
lib.OSGetTime.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.OSGetTime.restype = ctypes.c_int
lib.OSGetCount.argtypes = None
lib.OSGetCount.restype = ctypes.c_int

def OSWait(n):
	return lib.OSWait(ctypes.c_int(n))

def OSAttachTimer(scale, fct):
	return lib.OSAttachTimer(ctypes.c_int(scale), ctypes.CFUNCTYPE(None)(fct))

def OSDetachTimer(t):
	return lib.OSDetachTimer(t)

def OSGetTime(hrs, mins, secs, ticks):
	return lib.OSGetTime(ctypes.pointer(ctypes.c_int(hrs)), ctypes.pointer(ctypes.c_int(mins)), ctypes.pointer(ctypes.c_int(secs)), ctypes.pointer(ctypes.c_int(ticks)))

def OSGetCount():
	return lib.OSGetCount()

#=============================================================================

lib.SERInit.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.SERInit.restype = ctypes.c_int
lib.SERSendChar.argtypes = [ctypes.c_int, ctypes.c_char]
lib.SERSendChar.restype = ctypes.c_int
lib.SERSend.argtypes = [ctypes.c_int, ctypes.c_char_p]
lib.SERSend.restype = ctypes.c_int
lib.SERReceiveChar.argtypes = [ctypes.c_int]
lib.SERReceiveChar.restype = ctypes.c_char
lib.SERReceive.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
lib.SERReceive.restype = ctypes.c_int
lib.SERCheck.argtypes = [ctypes.c_int]
lib.SERCheck.restype = ctypes.c_bool
lib.SERFlush.argtypes = [ctypes.c_int]
lib.SERFlush.restype = ctypes.c_int
lib.SERClose.argtypes = [ctypes.c_int]
lib.SERClose.restype = ctypes.c_int

def SERInit(interface, baud, handshake):
	return lib.SERInit(ctypes.c_int(interface), ctypes.c_int(baud), ctypes.c_int(handshake))

def SERSendChar(interface, ch):
	return lib.SERSendChar(ctypes.c_int(interface), ctypes.c_char(ch))

def SERSend(interface, buf):
	return lib.SERSend(ctypes.c_int(interface), ctypes.c_char_p(buf.encode('ascii')))

def SERReceiveChar(interface):
	return lib.SERSend(ctypes.c_int(interface))

def SERReceive(interface, buf, size):
	return lib.SERReceive(ctypes.c_int(interface), ctypes.c_char_p(buf.encode('ascii')), ctypes.c_int(size))

def SERCheck(interface):
	return lib.SERCheck(ctypes.c_int(interface))

def SERFlush(interface):
	return lib.SERFlush(ctypes.c_int(interface))

def SERClose(interface):
	return lib.SERClose(ctypes.c_int(interface))

#=============================================================================

lib.AUBeep.argtypes = None
lib.AUBeep.restype = ctypes.c_int
lib.AUPlay.argtypes = [ctypes.POINTER(ctypes.c_char)]
lib.AUPlay.restype = ctypes.c_int
lib.AUDone.argtypes = None
lib.AUDone.restype = ctypes.c_int
lib.AUMicrophone.argtypes = None
lib.AUMicrophone.restype = ctypes.c_int

def AUBeep():
	return lib.AUBeep()

def AUDone():
	return lib.AUDone()

def AUPlay(filename):
	return lib.AUPlay(ctypes.c_char_p(filename.encode('ascii')))

def AUMicrophone():
	return lib.AUMicrophone()

#=============================================================================

if(Sim == True):
    lib.PSDGet.argtypes = [ctypes.c_int]
    lib.PSDGet.restype = ctypes.c_int
    lib.PSDGetRaw.argtypes = [ctypes.c_int]
    lib.PSDGetRaw.restype = ctypes.c_int
    lib.LIDARGet.argtypes = [ctypes.POINTER(ctypes.c_int)]
    lib.LIDARGet.restype = ctypes.POINTER(ctypes.c_int)
    lib.LIDARSet.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
    lib.LIDARSet.restype = ctypes.c_int

    def PSDGet(psd):
    	return lib.PSDGet(ctypes.c_int(psd))

    def PSDGetRaw(psd):
    	return lib.PSDGetRaw(ctypes.c_int(psd))

    def LIDARGet():
    	size = ctypes.c_int.in_dll(lib, "LIDAR_RESOLUTION").value
    	BUF = (ctypes.c_int*size)()
    	lib.LIDARGet(ctypes.cast(BUF, ctypes.POINTER(ctypes.c_int)))
    	scan = np.asarray(BUF, dtype=int)
    	scan = scan.tolist()
    	return scan

    def LIDARSet(angRange, tilt, numPoints):
    	lib.LIDARSet(ctypes.c_int(angRange), ctypes.c_int(tilt), ctypes.c_int(numPoints))
    	
#=============================================================================

lib.SERVOSet.argtypes = [ctypes.c_int, ctypes.c_int]
lib.SERVOSet.restype = ctypes.c_int
lib.SERVOSetRaw.argtypes = [ctypes.c_int, ctypes.c_int]
lib.SERVOSetRaw.restype = ctypes.c_int
lib.SERVORange.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.SERVORange.restype = ctypes.c_int
lib.MOTORDrive.argtypes = [ctypes.c_int, ctypes.c_int]
lib.MOTORDrive.restype = ctypes.c_int
lib.MOTORDriveRaw.argtypes = [ctypes.c_int, ctypes.c_int]
lib.MOTORDriveRaw.restype = ctypes.c_int
lib.MOTORPID.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.MOTORPID.restype = ctypes.c_int
#lib.MOTORPIDOff.argtypes = [ctypes.c_int]
#lib.MOTORPIDOff.restype = ctypes.c_int
lib.MOTORSpeed.argtypes = [ctypes.c_int, ctypes.c_int]
lib.MOTORSpeed.restype = ctypes.c_int
lib.ENCODERRead.argtypes = [ctypes.c_int]
lib.ENCODERRead.restype = ctypes.c_int
lib.ENCODERReset.argtypes = [ctypes.c_int]
lib.ENCODERReset.restype = ctypes.c_int

def SERVOSet(servo, angle):
	return lib.SERVOSet(ctypes.c_int(servo), ctypes.c_int(angle))

def SERVOSetRaw(servo, angle):
	return lib.SERVOSetRaw(ctypes.c_int(servo), ctypes.c_int(angle))

def SERVORange(servo, low, high):
	return lib.SERVORange(ctypes.c_int(servo), ctypes.c_int(low), ctypes.c_int(high))

def MOTORDrive(motor, speed):
	return lib.MOTORDrive(ctypes.c_int(motor), ctypes.c_int(speed))

def MOTORDriveRaw(motor, speed):
	return lib.MOTORDriveRaw(ctypes.c_int(motor), ctypes.c_int(speed))

def MOTORPID(motor, p, i, d):
	return lib.MOTORPID(ctypes.c_int(motor), ctypes.c_int(p), ctypes.c_int(i), ctypes.c_int(d))

#def MOTORPIDOff(motor):
#	return lib.MOTORPIDOff(ctypes.c_int(motor))

def MOTORSpeed(motor, speed):
	return lib.MOTORSpeed(ctypes.c_int(motor), ctypes.c_int(speed))

def ENCODERRead(quad):
	return lib.ENCODERRead(ctypes.c_int(quad))

def ENCODERReset(quad):
	return lib.ENCODERReset(ctypes.c_int(quad))

#=============================================================================

lib.VWSetSpeed.argtypes = [ctypes.c_int, ctypes.c_int]
lib.VWSetSpeed.restype = ctypes.c_int
lib.VWGetSpeed.argtypes = [ctypes.c_int, ctypes.c_int]
lib.VWGetSpeed.restype = ctypes.c_int
lib.VWSetPosition.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.VWSetPosition.restype = ctypes.c_int
lib.VWGetPosition.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
lib.VWGetPosition.restype = ctypes.c_int
lib.VWStraight.argtypes = [ctypes.c_int, ctypes.c_int]
lib.VWStraight.restype = ctypes.c_int
lib.VWTurn.argtypes = [ctypes.c_int, ctypes.c_int]
lib.VWTurn.restype = ctypes.c_int
lib.VWCurve.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.VWCurve.restype = ctypes.c_int
lib.VWDrive.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
lib.VWDrive.restype = ctypes.c_int
lib.VWRemain.argtypes = None
lib.VWRemain.restype = ctypes.c_int
lib.VWDone.argtypes = None
lib.VWDone.restype = ctypes.c_int
lib.VWWait.argtypes = None
lib.VWWait.restype = ctypes.c_int
lib.VWStalled.argtypes = None
lib.VWStalled.restype = ctypes.c_int

def VWDrive(dis, vel, spd):
	return lib.VWDrive(ctypes.c_int(dis), ctypes.c_int(vel), ctypes.c_int(spd))

def VWCurve(dist, angle, speed):
	return lib.VWCurve(ctypes.c_int(dist), ctypes.c_int(angle), ctypes.c_int(speed))

def VWTurn(ang, speed):
	return lib.VWTurn(ctypes.c_int(ang), ctypes.c_int(speed))

def VWStraight(dist, speed):
	return lib.VWStraight(ctypes.c_int(dist), ctypes.c_int(speed))

def VWSetSpeed(speed, angspeed):
	return lib.VWSetSpeed(ctypes.c_int(speed), ctypes.c_int(angspeed))

def VWSetPosition(x, y, phi):
	return lib.VWSetPosition(ctypes.c_int(x), ctypes.c_int(y), ctypes.c_int(phi))

def VWGetPosition():
	x = ctypes.c_int()
	y = ctypes.c_int()
	phi = ctypes.c_int()
	result = lib.VWGetPosition(ctypes.pointer(x), ctypes.pointer(y), ctypes.pointer(phi))
	return [x.value, y.value, phi.value]

def VWDone():
	return lib.VWDone()

def VWRemain():
	return lib.VWRemain()

def VWWait():
	return lib.VWWait()

def VWStalled():
	return lib.VWStalled()

#=============================================================================

lib.DIGITALSetup.argtypes = [ctypes.c_int, ctypes.c_char]
lib.DIGITALSetup.restype = ctypes.c_int
lib.DIGITALRead.argtypes = [ctypes.c_int]
lib.DIGITALRead.restype = ctypes.c_int
lib.DIGITALReadAll.argtypes = None
lib.DIGITALReadAll.restype = ctypes.c_int
lib.DIGITALWrite.argtypes = [ctypes.c_int, ctypes.c_int]
lib.DIGITALWrite.restype = ctypes.c_int
lib.ANALOGRead.argtypes = [ctypes.c_int]
lib.ANALOGRead.restype = ctypes.c_int
lib.ANALOGVoltage.argtypes = None
lib.ANALOGVoltage.restype = ctypes.c_int

def DIGITALSetup(io, direction):
	return lib.DIGITALSetup(ctypes.c_int(io), ctypes.c_char(direction))

def DIGITALRead(io):
	return lib.DIGITALRead(ctypes.c_int(io))

def DIGITALReadAll():
	return lib.DIGITALReadAll()

def DIGITALWrite(io, state):
	return lib.DIGITALWrite(ctypes.c_int(io), ctypes.c_int(state))

def ANALOGRead(channel):
	return lib.ANALOGRead(ctypes.c_int(channel))

def ANALOGVoltage():
	return lib.ANALOGVoltage()

#=============================================================================

if(Sim == True):
    lib.SIMSetRobot.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    lib.SIMSetRobot.restype = ctypes.c_int
    lib.SIMGetRobot.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
    lib.SIMGetRobot.restype = ctypes.c_int

    def SIMSetRobot(robot_id, x, y, z, phi):
        return lib.SIMSetRobot(ctypes.c_int(robot_id), ctypes.c_int(x), ctypes.c_int(y), ctypes.c_int(z), ctypes.c_int(phi))
        
    def SIMGetRobot(robot_id):
        x = ctypes.c_int()
        y = ctypes.c_int()
        z = ctypes.c_int()
        phi = ctypes.c_int()
        result = lib.SIMGetRobot(ctypes.c_int(robot_id), ctypes.pointer(x), ctypes.pointer(y), ctypes.pointer(z), ctypes.pointer(phi))
        return [x.value, y.value, z.value, phi.value]

#=============================================================================
