"""The simulator window (Tk)."""
import base64
import time
import tkinter as tk
from tkinter import filedialog, ttk

import cv2

import panels
from board import BIT_POSITIONS, CELL_PITCH_MM, CELLS, DOT_PITCH_MM, LINE_PITCH_MM, ROWS, Board, render_png
from controller import Controller
from world import SCENES, World

PX = 4.0          # screen pixels per mm on the board
LEFT_GUTTER = 70  # room for the line buttons
TOP = 12

LINE_BUTTONS = [f'ROW_{i}' for i in range(8)] + ['LINE_9']  # 9th button: not mapped in the code
KEYS = {'j': 'LEFT', 'k': 'CENTER', 'l': 'RIGHT', **{str(i + 1): name for i, name in enumerate(LINE_BUTTONS)}}
HELP = 'Keys: W/S move · A/D turn · J/K/L = LEFT/CENTER/RIGHT · 1–9 = line buttons (1 = ROW_0)'


def photo(img_bgr):
    ok, png = cv2.imencode('.png', img_bgr)
    return tk.PhotoImage(data=base64.b64encode(png.tobytes()).decode())


class App:
    def __init__(self, root, scene='Two-box test', real_speed=True, line_seconds=1.3):
        self.root = root
        self.world = World(scene)
        self.board = Board(line_seconds=line_seconds)
        self.ctl = Controller(self.world, self.board)
        self.real_speed = tk.BooleanVar(value=real_speed)
        self.shown = [[None] * CELLS for _ in range(ROWS)]   # what the canvas currently draws
        self.images = {}

        root.title('Virtual Canute')
        root.configure(padx=10, pady=8)
        self._build_board()
        self._build_front_buttons()
        self._build_panels()
        self._build_controls()
        self.moves = {'w': lambda: self.world.move(0.25), 's': lambda: self.world.move(-0.25),
                      'a': lambda: self.world.turn(15), 'd': lambda: self.world.turn(-15)}
        root.bind('<KeyPress>', self.on_key)
        self.refresh_world()
        self.refresh_panels()
        self.loop()

    # -- layout ----------------------------------------------------------------

    def _build_board(self):
        w = LEFT_GUTTER + int((CELLS * CELL_PITCH_MM + 4) * PX)
        h = TOP + int(ROWS * LINE_PITCH_MM * PX) + 6
        self.canvas = tk.Canvas(self.root, width=w, height=h, bg='#f4f2ec', highlightthickness=0)
        self.canvas.grid(row=0, column=0, columnspan=2, sticky='w')
        self.line_bg, self.dots = [], {}
        for line in range(ROWS):
            y0 = TOP + line * LINE_PITCH_MM * PX - 1.2 * PX
            self.line_bg.append(self.canvas.create_rectangle(
                LEFT_GUTTER - 4, y0, w - 2, y0 + 7.4 * PX, fill='', outline=''))
            yc = TOP + (line * LINE_PITCH_MM + DOT_PITCH_MM) * PX
            name = LINE_BUTTONS[line]
            tag = f'btn{line}'
            self.canvas.create_rectangle(8, yc - 9, LEFT_GUTTER - 14, yc + 9, fill='#d9d6cc', outline='#a29e92',
                                         tags=tag)
            self.canvas.create_text((8 + LEFT_GUTTER - 14) / 2, yc, text=name.replace('_', ' '),
                                    font=('Helvetica', 9), tags=tag)
            self.canvas.tag_bind(tag, '<Button-1>', lambda e, b=name: self.press(b))
            for cell in range(CELLS):
                for bit, (r, c) in BIT_POSITIONS.items():
                    self.dots[line, cell, bit] = self.canvas.create_oval(0, 0, 0, 0)
        self._draw_lines(range(ROWS))

    def _build_front_buttons(self):
        frame = ttk.Frame(self.root)
        frame.grid(row=1, column=0, columnspan=2, pady=(6, 2))
        for label, button in (('◀  LEFT  (J)', 'LEFT'), ('●  CENTER  (K)', 'CENTER'), ('RIGHT  ▶  (L)', 'RIGHT')):
            ttk.Button(frame, text=label, width=16, command=lambda b=button: self.press(b)).pack(side='left', padx=6)
        self.status = ttk.Label(self.root, font=('Helvetica', 12))
        self.status.grid(row=2, column=0, columnspan=2, sticky='w', pady=(4, 8))

    def _build_panels(self):
        left = ttk.LabelFrame(self.root, text='Room from above (camera red, 5 m view window blue)')
        left.grid(row=3, column=0, sticky='nw', padx=(0, 10))
        self.world_label = ttk.Label(left)
        self.world_label.pack(padx=4, pady=4)
        right = ttk.Frame(self.root)
        right.grid(row=3, column=1, sticky='nw')
        self.panel_labels = {}
        for key, title in (('crop', 'Map crop, /braille/heightmap (grey = 128, which gets raised)'),
                           ('sent', 'Sent to the board (red rows: the Canute never shows them)'),
                           ('camera', 'Camera → edges, /braille_image_debug (load a photo below)')):
            box = ttk.LabelFrame(right, text=title)
            box.pack(anchor='w', pady=(0, 4))
            self.panel_labels[key] = ttk.Label(box)
            self.panel_labels[key].pack(padx=4, pady=4)

    def _build_controls(self):
        bar = ttk.Frame(self.root)
        bar.grid(row=4, column=0, columnspan=2, sticky='we', pady=(8, 0))
        ttk.Label(bar, text='Scene').pack(side='left')
        self.scene = tk.StringVar(value=self.world.scene)
        box = ttk.Combobox(bar, textvariable=self.scene, values=list(SCENES), state='readonly', width=14)
        box.pack(side='left', padx=(4, 12))
        box.bind('<<ComboboxSelected>>', lambda e: self.load_scene())
        ttk.Button(bar, text='Load camera photo…', command=self.load_photo).pack(side='left')
        ttk.Checkbutton(bar, text='Real-speed refresh', variable=self.real_speed).pack(side='left', padx=12)
        ttk.Label(bar, text='s per changed line').pack(side='left')
        self.line_seconds = tk.DoubleVar(value=self.board.line_seconds)
        # Read-only (arrows only): typed digits would also press the line buttons.
        ttk.Spinbox(bar, from_=0.2, to=3.0, increment=0.1, width=4, textvariable=self.line_seconds,
                    command=self._set_line_seconds, state='readonly').pack(side='left', padx=(4, 12))
        ttk.Button(bar, text='Save board image…', command=self.save_png).pack(side='left')
        ttk.Label(self.root, text=HELP, foreground='#666').grid(row=5, column=0, columnspan=2, sticky='w', pady=(6, 0))

    # -- drawing ---------------------------------------------------------------

    def _draw_lines(self, lines):
        for line in lines:
            for cell in range(CELLS):
                value = self.board.cells[line][cell]
                if self.shown[line][cell] == value:
                    continue
                self.shown[line][cell] = value
                for bit, (r, c) in BIT_POSITIONS.items():
                    x = LEFT_GUTTER + (cell * CELL_PITCH_MM + c * DOT_PITCH_MM) * PX
                    y = TOP + (line * LINE_PITCH_MM + r * DOT_PITCH_MM) * PX
                    if value >> bit & 1:
                        rad = 0.8 * PX
                        self.canvas.coords(self.dots[line, cell, bit], x - rad, y - rad, x + rad, y + rad)
                        self.canvas.itemconfig(self.dots[line, cell, bit], fill='#222', outline='#222')
                    else:
                        rad = 0.35 * PX
                        self.canvas.coords(self.dots[line, cell, bit], x - rad, y - rad, x + rad, y + rad)
                        self.canvas.itemconfig(self.dots[line, cell, bit], fill='', outline='#b9b5aa')

    def _set_image(self, label, img):
        self.images[label] = photo(img)
        label.configure(image=self.images[label])

    def refresh_world(self):
        self._set_image(self.world_label, panels.world_view(self.world, self.ctl.size))

    def refresh_panels(self):
        self._set_image(self.panel_labels['crop'], panels.map_crop_view(self.ctl.map_crop))
        self._set_image(self.panel_labels['sent'], panels.sent_view(self.ctl.processed))
        self._set_image(self.panel_labels['camera'], panels.camera_view(self.ctl.camera_debug))

    def loop(self):
        now = time.monotonic()
        if self.board.tick(now, self.real_speed.get()):
            self._draw_lines(range(ROWS))
        active = self.board.refreshing_line()
        for line, item in enumerate(self.line_bg):
            self.canvas.itemconfig(item, fill='#ffe7a8' if line == active else '')
        if self.board.busy:
            refresh = f'Refreshing line {active + 1} · about {self.board.seconds_left(now):.0f} s left'
        else:
            refresh = 'Board idle'
        self.status.configure(text=f'View: {self.ctl.mode}   ·   {refresh}   ·   {self.ctl.message}')
        self.root.after(40, self.loop)

    # -- actions ---------------------------------------------------------------

    def on_key(self, event):
        key = event.keysym.lower()
        if key in KEYS:
            self.press(KEYS[key])
        elif key in self.moves:
            self.moves[key]()
            self.refresh_world()

    def press(self, button):
        self.ctl.press(button)
        self.refresh_panels()

    def load_scene(self):
        self.world.load(self.scene.get())
        self.ctl.message = f'Loaded "{self.world.scene}". Press CENTER in occupancy view to see it.'
        self.refresh_world()

    def load_photo(self):
        path = filedialog.askopenfilename(filetypes=[('Images', '*.png *.jpg *.jpeg')])
        if path:
            img = cv2.imread(path)
            if img is None:
                self.ctl.message = f'Could not read {path}'
            else:
                self.ctl.set_camera_image(img)
                self.ctl.message = 'Camera photo loaded. It reaches the board at the next update.'
                self.refresh_panels()

    def save_png(self):
        path = filedialog.asksaveasfilename(defaultextension='.png', filetypes=[('PNG', '*.png')])
        if path:
            render_png(self.board.cells, path)
            self.ctl.message = f'Saved {path}'

    def _set_line_seconds(self):
        try:
            self.board.line_seconds = float(self.line_seconds.get())
        except (tk.TclError, ValueError):
            pass
