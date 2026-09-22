import xml.etree.ElementTree as ET
import copy
from pathlib import Path

workspace_dir = Path(__file__).resolve().parent
file_path = workspace_dir / 'src/unitree_r1_description/urdf/r1.urdf'
tree = ET.parse(file_path)
root = tree.getroot()

for link in root.findall('link'):
    # 1. Удаляем сломанные коллизии
    for col in link.findall('collision'):
        link.remove(col)
    
    # 2. Проходим по визуалу и чиним его
    for visual in link.findall('visual'):
        if visual.find('origin') is None:
            orig = ET.Element('origin')
            orig.set('xyz', '0 0 0')
            orig.set('rpy', '0 0 0')
            visual.insert(0, orig)
        
        # 3. Правильное глубокое копирование тегов!
        collision = ET.Element('collision')
        collision.append(copy.deepcopy(visual.find('origin')))
        collision.append(copy.deepcopy(visual.find('geometry')))
        link.append(collision)

tree.write(file_path, encoding='utf-8', xml_declaration=True)
print("Успех: URDF починен, коллизии добавлены правильно!")
