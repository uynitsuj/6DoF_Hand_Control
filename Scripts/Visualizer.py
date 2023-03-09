
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import sys


class Visualizer(object):
    def __init__(self):
        self.lm3d = []
        self.traces = dict()
        self.app = QtWidgets.QApplication(sys.argv)
        self.w = gl.GLViewWidget()
        self.w.opts['distance'] = 20
        self.w.setWindowTitle('pyqtgraph Hand Pose')
        self.w.setGeometry(1000, 500, 800, 500)
        self.w.show()
        self.setup()

    def setup(self):
        gsz = 10
        gsp = 1
        gx = gl.GLGridItem(color=(255, 255, 255, 60))
        gx.setSize(gsz, gsz, gsz)
        gx.setSpacing(gsp, gsp, gsp)
        gx.rotate(90, 0, 1, 0)
        gx.translate(-gsz/2, 0, gsz/2)
        self.w.addItem(gx)
        gy = gl.GLGridItem(color=(255, 255, 255, 60))
        gy.setSize(gsz, gsz, gsz)
        gy.setSpacing(gsp, gsp, gsp)
        gy.rotate(90, 1, 0, 0)
        gy.translate(0, -gsz/2, gsz/2)
        self.w.addItem(gy)
        gz = gl.GLGridItem(color=(255, 255, 255, 100))
        gz.setSize(gsz, gsz, gsz)
        gz.setSpacing(gsp, gsp, gsp)
        self.w.addItem(gz)

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtWidgets.QApplication.instance().exec_()

    def update(self):
        del self.w.items[:]
        self.w.clear()
        self.setup()
        lm3dlist = self.lm3d.get()
        if lm3dlist:
            print(lm3dlist)
            width = 10

            # Thumb
            self.w.addItem(gl.GLLinePlotItem(
                pos=lm3dlist[0:5], color=pg.glColor((4, 100)), width=width, antialias=True))
            # Index
            self.w.addItem(gl.GLLinePlotItem(
                pos=np.append([lm3dlist[0]], lm3dlist[5:9], axis=0), color=pg.glColor((4, 100)), width=width, antialias=True))
            # Middle
            self.w.addItem(gl.GLLinePlotItem(
                pos=lm3dlist[9:13], color=pg.glColor((4, 100)), width=width, antialias=True))
            # Ring
            self.w.addItem(gl.GLLinePlotItem(
                pos=lm3dlist[13:17], color=pg.glColor((4, 100)), width=width, antialias=True))
            # Pinky
            self.w.addItem(gl.GLLinePlotItem(
                pos=np.append([lm3dlist[0]],
                              lm3dlist[17:21], axis=0), color=pg.glColor((4, 100)), width=width, antialias=True))
            # Knuckle
            knuckle = np.append([lm3dlist[5]], [lm3dlist[9]], axis=0)
            knuckle = np.append(knuckle, [lm3dlist[13]], axis=0)
            knuckle = np.append(knuckle, [lm3dlist[17]], axis=0)
            self.w.addItem(gl.GLLinePlotItem(
                pos=knuckle, color=pg.glColor((4, 100)), width=width, antialias=True))

    def animation(self, lm3d_q):
        self.lm3d = lm3d_q
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(1)
        self.start()
