from tkinter import *
from tkinter import messagebox
import time
import numpy as np
from PIL import ImageTk, Image, ImageDraw
import threading
from client import start_tcp_client, u16_to_v


class Display:
    FRAME_SIZE = 600

    def __init__(self, root=None):
        self.root = root if root is not None else Tk()
        self.root.title("展示WIFI数据")
        self.panel = Label(self.root)
        self.panel.pack(side=TOP, expand=1, fill=X)
        self.th_socket = threading.Thread(target=self.socket_thread)
        self.th_socket.setDaemon(True)
        self.th_socket.start()
        self.lock = threading.Lock()
        self.frames = [0 for _ in range(self.FRAME_SIZE)]
        self.show()

    def add_data(self, data):
        self.lock.acquire()
        self.frames.append(data)
        if len(self.frames) > self.FRAME_SIZE:
            self.frames = self.frames[1:]
        self.lock.release()
        self.show()

    def socket_thread(self):
        def handle(data: int):
            val = u16_to_v(data)
            print(val)
            self.add_data(val)

        def handle_data(data):
            s = data.decode()
            if len(s) == 0:
                return
            sp = s.split('\r\n')
            for p in sp:
                if len(p) == 0:
                    continue
                handle(int(p))

        start_tcp_client('192.168.137.240', 80, handler=handle_data)

    def update_image(self, im: Image):
        imp = ImageTk.PhotoImage(image=im)
        self.panel.configure(image=imp)
        self.panel.image = imp

    def draw(self):
        width = 1
        height = 32 *6
        colors = [
            'red', 'orange', 'yellow', 'green', 'cyan', 'blue', 'purple',
            'red', 'orange', 'yellow', 'green', 'cyan', 'blue', 'purple',
        ]

        size = (width * self.FRAME_SIZE, height)
        scalar = size[1] / 2 * 1.0
        im = Image.new("RGB", size, color='white')
        draw = ImageDraw.Draw(im)
        self.lock.acquire()
        for i in range(self.FRAME_SIZE - 2):
            draw.line((width * i, size[1] - self.frames[i] * scalar,
                       width * (i + 1), size[1] - self.frames[i + 1] * scalar), fill='red')
        self.lock.release()
        # sx = size[0] - width * self.select
        # draw.line((sx, 0, sx, size[1]), fill='red')
        return im

    def show(self):
        self.update_image(self.draw())

    def mainloop(self):
        self.root.mainloop()


if __name__ == '__main__':
    display = Display()
    display.mainloop()
