# Проверка провода камера -> робот (гиробот).
# Шлёт в UART3 (вывод P4 -> D12 Nano) строку "cam N" раз в 0.5 с и мигает зелёным.
# Робот пересылает принятое в телеметрию строками "cam> cam N".
import time
from machine import LED, UART

uart = UART(3, 9600)
led = LED("LED_GREEN")

n = 0
while True:
    n += 1
    uart.write("cam %d\n" % n)
    print("отправлено: cam", n)
    led.toggle()
    time.sleep_ms(500)
