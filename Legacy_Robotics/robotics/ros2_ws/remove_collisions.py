import xml.etree.ElementTree as ET
from pathlib import Path

workspace_dir = Path(__file__).resolve().parent
file_path = workspace_dir / 'src/unitree_r1_description/urdf/r1.urdf'
tree = ET.parse(file_path)
root = tree.getroot()

for link in root.findall('link'):
    for col in link.findall('collision'):
        link.remove(col)

tree.write(file_path, encoding='utf-8', xml_declaration=True)
print("Успех: Взрывоопасные коллизии удалены. Файл чист!")
