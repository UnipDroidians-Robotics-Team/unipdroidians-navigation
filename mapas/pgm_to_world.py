import yaml
from PIL import Image

with open('udh1_mapa.yaml', 'r') as f:
    meta = yaml.safe_load(f)

resolution = meta['resolution']
origin = meta['origin']

img = Image.open('udh1_mapa.pgm').convert('L')
width, height = img.size
pixels = img.load()

walls = []
for y in range(height):
    for x in range(width):
        if pixels[x, y] < 128:
            wx = origin[0] + x * resolution
            wy = origin[1] + (height - y) * resolution
            walls.append((wx, wy))

world = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="udh1_world">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
"""

for i, (wx, wy) in enumerate(walls):
    world += f"""
    <model name="wall_{i}">
      <static>true</static>
      <pose>{wx} {wy} 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>{resolution} {resolution} 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>{resolution} {resolution} 1</size></box></geometry></visual>
      </link>
    </model>"""

world += "\n  </world>\n</sdf>"

with open('udh1_mapa.world', 'w') as f:
    f.write(world)

print("Concluído! udh1_mapa.world gerado.")
