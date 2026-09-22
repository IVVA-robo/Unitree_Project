import xml.etree.ElementTree as ET
from pathlib import Path

workspace_dir = Path(__file__).resolve().parent
file_path = workspace_dir / 'src/unitree_r1_description/urdf/r1.urdf'
tree = ET.parse(file_path)
root = tree.getroot()

for link in root.findall('link'):
    # Если коллизии нет, но есть графика — создаем копию
    if link.find('collision') is None and link.find('visual') is not None:
        visual = link.find('visual')
        collision = ET.Element('collision')
        for child in visual:
            collision.append(child)
        link.append(collision)

tree.write(file_path, encoding='utf-8', xml_declaration=True)
print("Успех: Физические границы (коллизии) добавлены во все детали!")
