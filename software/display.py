from tkinter import *
# from tkinter import messagebox
import time
import math
# import random
# import numpy as np
from PIL import ImageTk, Image, ImageDraw, ImageFont
import threading
from client import start_tcp_client, u16_to_v

M = 1000000


class Display:
    WIDTH_UNIT = 30
    RANGE_UNIT = 40
    HEIGHT = 700
    # VAL_MAX = 3.5
    VAL_MAX = 1.2
    FONT = ImageFont.truetype('C:/windows/fonts/Dengl.ttf', 20)
    REMOTE_SOCKET_HOST = "192.168.137.70"
    REMOTE_SOCKET_PORT = 80
    DISPLAY_VALUE_STEP = 0.1

    def __init__(self, root=None):
        self.root = root if root is not None else Tk()
        self.root.title("展示WIFI数据")
        self.panel = Label(self.root)
        self.panel.pack(side=TOP, expand=1, fill=X)
        self.lock = threading.Lock()
        self.frames: list = []  # [[0, 0] for _ in range(self.FRAME_SIZE)]
        self.show()

        self.th_socket = threading.Thread(target=self.socket_thread)
        self.th_socket.setDaemon(True)
        self.th_socket.start()

    def add_data(self, data):
        self.lock.acquire()
        # self.frames.append(data)
        # self.frames.sort(key=lambda x: x[1])
        # used = set({})
        # for i in range(len(self.frames)):
        #     if self.frames[i] is None:
        #         continue
        #     if self.frames[i][1] in used:
        #         self.frames[i] = None
        #         continue
        #     used.add(self.frames[i][1])
        # self.frames = [f for f in self.frames if f is not None]

        for i in range(len(self.frames)):
            if self.frames[i] is None:
                continue
            if self.frames[i][1] == data[1]:
                self.frames[i] = None
                continue
        self.frames = [f for f in self.frames if f is not None]
        self.frames.append(data)
        self.frames.sort(key=lambda x: x[1])

        # if len(self.frames) == self.RANGE_UNIT:
        #     max_val = max

        # if len(self.frames) > self.FRAME_SIZE:
        #     print(f'add: {data}, pop: {self.frames[0]}')
        #     self.frames = self.frames[1:]
        self.lock.release()
        self.show()

    def socket_thread(self):
        def handle(data: list):
            out = u16_to_v(data[0])
            if data[1] > M * self.RANGE_UNIT:
                return
            real = ((math.pow(10, 2 * out - 3)) * 0.039899) / math.sqrt(2)
            print(data, out, real)
            self.add_data([real, data[1] / M])

        def handle_data(data):
            s = data.decode()
            if len(s) == 0:
                return
            sp = s.split('\r\n')
            for p in sp:
                if len(p) == 0:
                    continue
                if ',' not in p:
                    continue
                try:
                    handle([int(i) for i in p.split(",")])
                except ValueError as e:
                    print(e)

        start_tcp_client(self.REMOTE_SOCKET_HOST, self.REMOTE_SOCKET_PORT, handler=handle_data)

        # while True:
        #     for i in range(40):
        #         handle([65525 / 2 + i * 1000, i * M])
        #         # handle([65535 * i / 400, i / 10])
        #         time.sleep(0.01)

    def update_image(self, im: Image):
        imp = ImageTk.PhotoImage(image=im)
        self.panel.configure(image=imp)
        self.panel.image = imp

    def draw(self):
        size = (self.WIDTH_UNIT * self.RANGE_UNIT, self.HEIGHT)
        scalar = size[1] / self.VAL_MAX
        im = Image.new("RGB", size, color='black')
        draw = ImageDraw.Draw(im)
        self.lock.acquire()
        # self.frames.sort(key=lambda x: x[1])
        for i in range(len(self.frames) - 1):
            data = self.frames[i]
            nxt = self.frames[i + 1]
            rect = (self.WIDTH_UNIT * data[1], size[1] - data[0] * scalar,
                    self.WIDTH_UNIT * nxt[1], size[1] - nxt[0] * scalar)
            # if rect[0] != 0 and len(self.frames) == self.FRAME_SIZE:
            #     print(f'{data[0]} => {nxt[0]} rect: {rect}')
            # print(data[1])
            draw.line(rect, fill='green', width=4)
        self.lock.release()
        # sx = size[0] - width * self.select
        # draw.line((sx, 0, sx, size[1]), fill='white')
        return im

    def add_axes(self, im: Image):
        offset = [50, 40]
        size = [im.size[0] + offset[0], im.size[1] + offset[1]]
        base = Image.new("RGB", size, color='black')
        base.paste(im, (offset[0], 0))
        draw = ImageDraw.Draw(base)
        draw.line((offset[0] - 2, 0, offset[0] - 2, im.size[1]), fill='white', width=2)
        draw.line((offset[0] - 2, im.size[1] + 1, size[0], im.size[1] + 1), fill='white', width=2)
        draw.text((55, 5), text="U/V", fill='white', font=self.FONT)
        draw.text((size[0] - 90, im.size[1] - 40), text="freq/MHz", fill='white', font=self.FONT)
        for i in range(0, self.RANGE_UNIT, 5):
            draw.line((offset[0] + i * self.WIDTH_UNIT, size[1] - offset[1] + 2,
                       offset[0] + i * self.WIDTH_UNIT, size[1] - offset[1] + 5), fill='white', width=2)
            draw.text((offset[0] + i * self.WIDTH_UNIT, size[1] - offset[1] + 10), text=f"{i}", fill='white',
                      font=self.FONT)
        val = 0
        while val < self.VAL_MAX:
            draw.line((offset[0] - 5, im.size[1] * val / self.VAL_MAX,
                       offset[0] - 1, im.size[1] * val / self.VAL_MAX), fill='white', width=2)
            draw.text((offset[0] - 35, im.size[1] * val / self.VAL_MAX), text="%.1f" % (self.VAL_MAX - val),
                      fill='white', font=self.FONT)
            val += self.DISPLAY_VALUE_STEP
        return base

    def show(self):
        self.update_image(self.add_axes(self.draw()))

    def mainloop(self):
        self.root.mainloop()


if __name__ == '__main__':
    display = Display()
    display.mainloop()
