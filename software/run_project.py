import time
from machine import I2C, Pin, ADC, UART
from esp8266_i2c_lcd import I2cLcd

try:
    import usocket as socket
except:
    import socket


def main():
    freq = 0
    i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)
    adc = ADC(0)
    lcd = I2cLcd(i2c, 0x27, 2, 16)
    lcd.putstr("starting\nproject")
    time.sleep(2)
    lcd.clear()
    s = socket.socket()
    ai = socket.getaddrinfo("0.0.0.0", 80)
    addr = ai[0][-1]
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(5)
    lcd.putstr("Listening:80\n")
    '''
    while True:
        res = s.accept()
        client_sock = res[0]
        client_addr = res[1]
        print("Client address:", client_addr)
        lcd.putstr("%s" % client_addr[0])
        print("Client socket:", client_sock)
        client_stream = client_sock
        while True:
            data = input()
            print("to send data", data)
            client_stream.write(data)
        client_stream.close()
    '''

    while True:
        conn, addr = s.accept()
        print('Connected by', addr)
        while True:
            lcd.clear()
            freq = int(input())
            val = adc.read_u16()
            lcd.putstr("%s\n%s" % (val, freq))
            print('data', val)
            try:
                conn.sendall(("%s,%s\r\n" % (val, freq)).encode())
            except Exception as e:
                print('exception', e)
                break
            time.sleep_ms(20)


main()



