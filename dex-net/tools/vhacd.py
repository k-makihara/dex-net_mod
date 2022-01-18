import pybullet as p
import pybullet_data as pd
import os

import pybullet_data

p.connect(p.DIRECT)

name_in = []
name_out = []
name_log = []

name_in.append("/home/user/Downloads/IROS2022_dex-fractal-20211224T053701Z-001/IROS2022_dex-fractal/3dfractal_render_modern_opengl/3dfractal_render_modern_opengl/testply/test.obj")
name_out.append("/home/user/Downloads/IROS2022_dex-fractal-20211224T053701Z-001/IROS2022_dex-fractal/3dfractal_render_modern_opengl/3dfractal_render_modern_opengl/testply/test_vhacd.obj")
name_log.append("/home/user/Downloads/IROS2022_dex-fractal-20211224T053701Z-001/IROS2022_dex-fractal/3dfractal_render_modern_opengl/3dfractal_render_modern_opengl/testply/log.txt")

for i in range(len(name_in)):
    p.vhacd(name_in[i], name_out[i], name_log[i])

