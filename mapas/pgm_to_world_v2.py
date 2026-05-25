import yaml
from PIL import Image

with open('udh1_mapa.yaml', 'r') as f:
    meta = yaml.safe_load(f)

resolution = meta['resolution']
origin = meta['origin']

img = Image.open('udh1_mapa.pgm').convert('L')
width, height = img.size
pixels = img.load()

# Agrupa linhas horizontais contíguas em paredes maiores
walls = []
for y in range(height):
    x = 0
    while x < width:
        if pixels[x, y] < 128:
            start = x
            while x < width and pixels[x, y] < 128:
                x += 1
            length = x - start
            cx = origin[0] + (start + length / 2) * resolution
            cy = origin[1] + (height - y) * resolution
            walls.append((cx, cy, length * resolution, resolution))
        else:
            x += 1

world = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="udh1_world">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
"""

for i, (cx, cy, lx, ly) in enumerate(walls):
    world += f"""
    <model name="wall_{i}">
      <static>true</static>
      <pose>{cx} {cy} 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>{lx} {ly} 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>{lx} {ly} 1</size></box></geometry></visual>
      </link>
    </model>"""

world += "\n  </world>\n</sdf>"

with open('udh1_mapa.world', 'w') as f:
    f.write(world)

print(f"Concluído! {len(walls)} paredes geradas.")
