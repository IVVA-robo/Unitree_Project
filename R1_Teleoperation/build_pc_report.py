from docx import Document
from docx.shared import Inches, Pt, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.enum.style import WD_STYLE_TYPE
from docx.enum.text import WD_LINE_SPACING
from docx.opc.constants import RELATIONSHIP_TYPE as RT
from docx.enum.section import WD_ORIENT
from pathlib import Path

OUT = Path('/home/unitree/Документы/ChatGPT/Unitree r1 Ultra-Lightweight/ПК для Unitree R1 отчёт.docx')

def set_cell_shading(cell, fill):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = tcPr.find(qn('w:shd'))
    if shd is None:
        shd = OxmlElement('w:shd'); tcPr.append(shd)
    shd.set(qn('w:fill'), fill)

def set_cell_border(cell, **kwargs):
    tc = cell._tc; tcPr = tc.get_or_add_tcPr()
    tcBorders = tcPr.first_child_found_in('w:tcBorders')
    if tcBorders is None:
        tcBorders = OxmlElement('w:tcBorders'); tcPr.append(tcBorders)
    for edge in ('top','left','bottom','right','insideH','insideV'):
        if edge in kwargs:
            tag = 'w:{}'.format(edge); element = tcBorders.find(qn(tag))
            if element is None:
                element = OxmlElement(tag); tcBorders.append(element)
            for k,v in kwargs[edge].items(): element.set(qn('w:'+k), str(v))

def set_cell_margins(cell, top=100, start=110, bottom=100, end=110):
    tc = cell._tc; tcPr = tc.get_or_add_tcPr(); tcMar = tcPr.first_child_found_in('w:tcMar')
    if tcMar is None:
        tcMar = OxmlElement('w:tcMar'); tcPr.append(tcMar)
    for m,v in [('top',top),('start',start),('bottom',bottom),('end',end)]:
        node = tcMar.find(qn('w:'+m))
        if node is None: node=OxmlElement('w:'+m); tcMar.append(node)
        node.set(qn('w:w'), str(v)); node.set(qn('w:type'),'dxa')

def set_repeat_table_header(row):
    trPr = row._tr.get_or_add_trPr(); tblHeader = OxmlElement('w:tblHeader'); tblHeader.set(qn('w:val'),'true'); trPr.append(tblHeader)

def set_row_cant_split(row):
    trPr = row._tr.get_or_add_trPr()
    cant = OxmlElement('w:cantSplit'); cant.set(qn('w:val'), 'true'); trPr.append(cant)

def set_width(cell, inches):
    tcPr = cell._tc.get_or_add_tcPr(); tcW = tcPr.find(qn('w:tcW'))
    if tcW is None: tcW=OxmlElement('w:tcW'); tcPr.append(tcW)
    tcW.set(qn('w:w'), str(int(inches*1440))); tcW.set(qn('w:type'),'dxa')

def add_hyperlink(paragraph, text, url, color='0563C1', underline=True):
    part = paragraph.part; r_id = part.relate_to(url, RT.HYPERLINK, is_external=True)
    hyperlink = OxmlElement('w:hyperlink'); hyperlink.set(qn('r:id'), r_id)
    new_run = OxmlElement('w:r'); rPr = OxmlElement('w:rPr')
    c = OxmlElement('w:color'); c.set(qn('w:val'), color); rPr.append(c)
    if underline:
        u = OxmlElement('w:u'); u.set(qn('w:val'),'single'); rPr.append(u)
    new_run.append(rPr); t = OxmlElement('w:t'); t.text=text; new_run.append(t); hyperlink.append(new_run)
    paragraph._p.append(hyperlink); return hyperlink

def add_page_field(paragraph):
    run=paragraph.add_run(); fld=OxmlElement('w:fldSimple'); fld.set(qn('w:instr'),'PAGE'); run._r.addnext(fld)

def add_bookmark(paragraph, name, bid):
    start=OxmlElement('w:bookmarkStart'); start.set(qn('w:id'),str(bid)); start.set(qn('w:name'),name)
    end=OxmlElement('w:bookmarkEnd'); end.set(qn('w:id'),str(bid)); paragraph._p.insert(0,start); paragraph._p.append(end)

def add_internal_link(paragraph, text, anchor):
    hyperlink=OxmlElement('w:hyperlink'); hyperlink.set(qn('w:anchor'),anchor)
    r=OxmlElement('w:r'); rPr=OxmlElement('w:rPr'); c=OxmlElement('w:color'); c.set(qn('w:val'),'0563C1'); rPr.append(c); u=OxmlElement('w:u'); u.set(qn('w:val'),'single'); rPr.append(u); r.append(rPr); t=OxmlElement('w:t'); t.text=text; r.append(t); hyperlink.append(r); paragraph._p.append(hyperlink)

def add_bold_label_para(doc, label, text):
    p=doc.add_paragraph(); p.paragraph_format.space_after=Pt(4); r=p.add_run(label); r.bold=True; p.add_run(text); return p

def add_source_line(doc, title, url, kind, accessed='28.09.2026'):
    p=doc.add_paragraph(style='Source'); p.add_run(f'{kind}: {title} (доступ {accessed}) - '); add_hyperlink(p, url, url); return p

def add_table(doc, headers, rows, widths=None, font_size=9):
    table=doc.add_table(rows=1, cols=len(headers)); table.alignment=WD_TABLE_ALIGNMENT.CENTER; table.autofit=False
    table.style='Table Grid'
    hdr=table.rows[0]; set_repeat_table_header(hdr); set_row_cant_split(hdr)
    for i,h in enumerate(headers):
        c=hdr.cells[i]; set_cell_shading(c,'404040'); set_cell_margins(c); c.vertical_alignment=WD_CELL_VERTICAL_ALIGNMENT.CENTER
        if widths: set_width(c,widths[i])
        p=c.paragraphs[0]; p.alignment=WD_ALIGN_PARAGRAPH.CENTER; p.paragraph_format.space_after=Pt(0)
        r=p.add_run(str(h)); r.bold=True; r.font.color.rgb=RGBColor(255,255,255); r.font.size=Pt(font_size)
        set_cell_border(c, top={'val':'single','sz':'6','color':'D9D9D9'},bottom={'val':'single','sz':'6','color':'D9D9D9'},left={'val':'single','sz':'6','color':'D9D9D9'},right={'val':'single','sz':'6','color':'D9D9D9'})
    for ridx,row in enumerate(rows):
        new_row=table.add_row(); set_row_cant_split(new_row); cells=new_row.cells
        for i,val in enumerate(row):
            c=cells[i]; set_cell_margins(c); c.vertical_alignment=WD_CELL_VERTICAL_ALIGNMENT.CENTER
            if widths: set_width(c,widths[i])
            if ridx%2==1: set_cell_shading(c,'F2F2F2')
            p=c.paragraphs[0]; p.paragraph_format.space_after=Pt(0); p.paragraph_format.line_spacing=1.0
            # support simple hyperlinks as tuples
            if isinstance(val, tuple):
                add_hyperlink(p,val[0],val[1])
            else:
                p.add_run(str(val))
            for rr in p.runs: rr.font.size=Pt(font_size)
            set_cell_border(c, top={'val':'single','sz':'6','color':'D9D9D9'},bottom={'val':'single','sz':'6','color':'D9D9D9'},left={'val':'single','sz':'6','color':'D9D9D9'},right={'val':'single','sz':'6','color':'D9D9D9'})
    return table

def add_heading(doc, text, level=1, anchor=None):
    p=doc.add_heading(text, level=level)
    if anchor: add_bookmark(p, anchor, add_heading.bid); add_heading.bid+=1
    p.paragraph_format.keep_with_next=True
    return p
add_heading.bid=10

# document setup
doc=Document(); sec=doc.sections[0]
sec.page_width=Inches(8.27); sec.page_height=Inches(11.69)
sec.top_margin=Inches(0.72); sec.bottom_margin=Inches(0.65); sec.left_margin=Inches(0.75); sec.right_margin=Inches(0.65)
# styles
styles=doc.styles
styles['Normal'].font.name='DejaVu Sans'; styles['Normal']._element.rPr.rFonts.set(qn('w:eastAsia'),'DejaVu Sans'); styles['Normal'].font.size=Pt(10.2)
styles['Normal'].paragraph_format.space_after=Pt(6); styles['Normal'].paragraph_format.line_spacing=1.08
for sname,size in [('Title',25),('Heading 1',17),('Heading 2',13),('Heading 3',11)]:
    st=styles[sname]; st.font.name='DejaVu Sans'; st._element.rPr.rFonts.set(qn('w:eastAsia'),'DejaVu Sans'); st.font.color.rgb=RGBColor(0,0,0); st.font.size=Pt(size); st.font.bold=True
styles['Title'].paragraph_format.space_after=Pt(14)
# LibreOffice/Word may carry a built-in blue bottom border on the Title style.
# Remove it explicitly so the cover uses whitespace and typography only.
for _style_name in ['Title']:
    _ppr = styles[_style_name]._element.get_or_add_pPr()
    _pbdr = _ppr.find(qn('w:pBdr'))
    if _pbdr is not None:
        _ppr.remove(_pbdr)
styles['Heading 1'].paragraph_format.space_before=Pt(15); styles['Heading 1'].paragraph_format.space_after=Pt(7)
styles['Heading 2'].paragraph_format.space_before=Pt(10); styles['Heading 2'].paragraph_format.space_after=Pt(4)
# custom source style
if 'Source' not in styles:
    st=styles.add_style('Source', WD_STYLE_TYPE.PARAGRAPH)
else: st=styles['Source']
st.font.name='DejaVu Sans'; st._element.rPr.rFonts.set(qn('w:eastAsia'),'DejaVu Sans'); st.font.size=Pt(8.6); st.font.color.rgb=RGBColor(65,65,65); st.paragraph_format.space_after=Pt(3); st.paragraph_format.left_indent=Inches(0.12)
# header/footer
header=sec.header.paragraphs[0]; header.alignment=WD_ALIGN_PARAGRAPH.RIGHT; hr=header.add_run('Unitree R1 | рабочая станция'); hr.font.size=Pt(8); hr.font.color.rgb=RGBColor(90,90,90)
footer=sec.footer.paragraphs[0]; footer.alignment=WD_ALIGN_PARAGRAPH.CENTER; fr=footer.add_run('ПК для Unitree R1  |  '); fr.font.size=Pt(8); add_page_field(footer)

# cover
p=doc.add_paragraph(style='Title'); p.alignment=WD_ALIGN_PARAGRAPH.CENTER; p.add_run('Рабочий ПК для Unitree R1')
p=doc.add_paragraph(); p.alignment=WD_ALIGN_PARAGRAPH.CENTER; r=p.add_run('Обоснование конфигурации для ROS2 Gazebo Isaac Sim и VR телеприсутствия'); r.bold=True; r.font.size=Pt(14); r.font.color.rgb=RGBColor(55,55,55)
p=doc.add_paragraph(); p.alignment=WD_ALIGN_PARAGRAPH.CENTER; p.paragraph_format.space_before=Pt(34); p.add_run('Подготовлено 28 сентября 2026 года').font.size=Pt(11)
p=doc.add_paragraph(); p.alignment=WD_ALIGN_PARAGRAPH.CENTER; p.paragraph_format.space_before=Pt(22); p.add_run('Бюджет: до 1 200 000 ₽  |  Регион покупки: Ростовская область').font.size=Pt(11)
doc.add_paragraph('')
p=doc.add_paragraph(); p.paragraph_format.space_before=Pt(50); p.paragraph_format.space_after=Pt(8); r=p.add_run('Решение'); r.bold=True; r.font.size=Pt(12)
p=doc.add_paragraph('Сбалансированная конфигурация на Ryzen 9 9950X3D, GeForce RTX 5090 32 ГБ и 128 ГБ DDR5 с двумя NVMe SSD. Она оставляет запас по бюджету, поддерживает VR-телеприсутствие и Isaac Sim, а отдельный Ethernet можно использовать для Unitree R1.')
p=doc.add_paragraph('Цены в отчёте являются наблюдением по независимым карточкам Яндекс Маркета на дату подготовки. Перед оплатой нужно повторно проверить наличие, ревизию и итоговую цену.')
doc.add_page_break()

# TOC
add_heading(doc,'Оглавление',1,'toc')
toc=[('1. Назначение и требования','s1'),('2. Краткий итоговый вариант','s2'),('3. Таблица комплектующих и цен','s3'),('4. Стоимость и запас бюджета','s4'),('5. Обоснование компонентов','s5'),('6. Видеокарта для Isaac Sim и VR','s6'),('7. RTX 5090 и RTX 4090','s7'),('8. Совместимость','s8'),('9. Питание охлаждение и шум','s9'),('10. Подключение Unitree R1 по Ethernet','s10'),('11. Модернизация','s11'),('12. Более дешёвая альтернатива','s12'),('13. Итоговая рекомендация','s13'),('14. Источники','s14')]
for label,anchor in toc:
    p=doc.add_paragraph(); p.paragraph_format.left_indent=Inches(0.18); add_internal_link(p,label,anchor)
doc.add_paragraph('Типы источников в тексте: официальные требования производителей и проектов, независимые тесты, практические обсуждения сообщества и инженерная рекомендация для данного сценария.')
doc.add_page_break()

# 1
add_heading(doc,'1. Назначение и требования',1,'s1')
doc.add_paragraph('Рабочая станция предназначена для разработки и эксплуатации программного стека Unitree R1: ROS2, Gazebo, RViz, Isaac Sim и Isaac Lab, обработки камер и CUDA, VR-телеприсутствия, Unity/Unreal, записи rosbag, видео и диагностических данных. Главный практический сценарий - одновременно держать симуляцию, вид робота от первого лица, VR-рендеринг, IDE, браузер и инструменты мониторинга.')
doc.add_paragraph('ROS2 и Gazebo сами по себе не требуют RTX 5090. Для них важнее стабильный процессор, быстрая память, SSD и корректно настроенная сеть. Большая видеокарта нужна из-за Isaac Sim, RTX-рендеринга, камер робота, VR, компьютерного зрения и CUDA. Этот отчёт поэтому разделяет базовые потребности ROS2/Gazebo и тяжёлые графические и обучающие задачи.')
doc.add_paragraph('Ограничение бюджета - 1 200 000 ₽ вместе с монитором и ИБП. Физический робот и его приводные цепи в стоимость не входят. ИБП предназначен для ПК и сетевого оборудования, а не для питания робота.')

# 2
add_heading(doc,'2. Краткий итоговый вариант',1,'s2')
add_table(doc,['Узел','Выбранное решение','Почему'],[
('CPU','AMD Ryzen 9 9950X3D, 16 ядер','Высокая интерактивность и сильная многопоточная работа ROS2, сборки и симуляции'),
('GPU','MSI GeForce RTX 5090 Ventus 3X, 32 ГБ','Запас VRAM для Isaac Sim, VR, камер и CUDA'),
('Память','Kingston FURY Beast 128 ГБ, 4x32 ГБ DDR5-5200','Практичный максимум AM5 в данном бюджете'),
('Диски','Samsung 990 PRO 2 ТБ + WD Black SN850X 4 ТБ','Разделение ОС/ПО и rosbag, моделей, видео'),
('Связь с R1','Встроенный LAN платы, отдельный порт под private subnet','Не требует отдельной карты; сеть управления отделяется от NoMachine и VR'),
('Итог','1 151 000 ₽ с монитором и ИБП','49 000 ₽ резерва до лимита')], widths=[1.15,2.1,3.95], font_size=9)
doc.add_paragraph('Инженерное решение: не переплачивать за экстремальную плату ASUS ROG Crosshair X870E Hero. TUF Gaming X870E-PLUS WIFI7 даёт нужные PCIe, M.2, DDR5 и сеть, а деньги направлены в RTX 5090, SSD и ИБП.')

# 3
add_heading(doc,'3. Таблица комплектующих и цен',1,'s3')
parts=[
('Процессор','AMD Ryzen 9 9950X3D OEM','70 129','Яндекс Маркет'),('Материнская плата','ASUS TUF Gaming X870E-PLUS WIFI7','42 777','Яндекс Маркет'),('ОЗУ','Kingston FURY Beast 128 ГБ 4x32 DDR5-5200','235 718','Яндекс Маркет'),('Видеокарта','MSI GeForce RTX 5090 Ventus 3X 32 ГБ GDDR7','571 119','Яндекс Маркет'),('Системный SSD','Samsung 990 PRO 2 ТБ','31 504','Яндекс Маркет'),('SSD данных','WD Black SN850X 4 ТБ','64 331','Яндекс Маркет'),('Блок питания','DeepCool GamerStorm PQ1200G 1200 Вт ATX 3.1','13 570','Яндекс Маркет'),('Охлаждение','ARCTIC Liquid Freezer III Pro 360','11 349','Яндекс Маркет'),('Корпус','Lian Li Lancool III Black','17 302','Яндекс Маркет'),('Доп. вентиляторы','Не обязательны: 4x140 мм уже в корпусе','0','Инженерная оценка'),('Сеть R1','Встроенный Ethernet материнской платы','0','Инженерная оценка'),('ИБП','APC BX1600MI-GR 1600 VA / 900 Вт','30 323','Яндекс Маркет'),('Кабели и крепления','Резерв на 12V-2x6, Ethernet, стяжки','3 000','Резерв'),('Монитор','LG UltraGear 32GR93U-B 4K 144 Гц','59 878','Яндекс Маркет')]
add_table(doc,['Компонент','Точная модель','Цена','Основание'],parts,widths=[1.25,3.45,0.85,1.55],font_size=8.7)

# 4
add_heading(doc,'4. Стоимость и запас бюджета',1,'s4')
add_table(doc,['Показатель','Сумма'],[('Комплектующие и монитор','1 151 000 ₽'),('Лимит','1 200 000 ₽'),('Остаток','49 000 ₽'),('Опционально 2x Noctua NF-A14','примерно 6 860 ₽; остаток будет около 42 140 ₽')],widths=[5.2,1.9],font_size=9.5)
doc.add_paragraph('Наиболее нестабильные позиции - RTX 5090 и комплект 128 ГБ DDR5. Если видеокарта станет дороже примерно 600 000 ₽, нужно заново пересчитать бюджет: ближайший рациональный шаг - RTX 4090 либо временная покупка меньшей карты с последующим обновлением. Нельзя считать цену наблюдения гарантией цены в магазине.')

# 5
add_heading(doc,'5. Обоснование компонентов',1,'s5')
components=[
('5.1 Процессор','AMD Ryzen 9 9950X3D OEM - 70 129 ₽. Он даёт 16 ядер и 32 потока, что полезно для компиляции ROS2, контейнеров, обработки камер, запуска нескольких узлов и CPU-части симуляции. 3D V-Cache помогает интерактивным нагрузкам и игровому/VR-рендеру, но не превращает Gazebo в GPU-задачу. Выбор оправдан балансом производительности и цены. Более дешёвый Ryzen 9 9950X обычно рационален для полностью многопоточной работы; Threadripper Pro с ECC RDIMM лучше для большого числа параллельных окружений, но платформа и память выводят его за бюджет. OEM требует отдельного кулера и проверки гарантии.'),
('5.2 Материнская плата','ASUS TUF Gaming X870E-PLUS WIFI7 - 42 777 ₽. Плата AM5 поддерживает Ryzen 9000, DDR5, несколько M.2 и PCIe 5.0, имеет быстрый LAN и Wi-Fi 7. Для R1 отдельная сетевая карта не нужна: один физический порт можно закрепить за private subnet робота. Более дорогая Crosshair X870E Hero даёт расширенные VRM, отладку и экстремальный разгон, но не ускоряет ROS2 пропорционально цене. Ограничение: четыре модуля памяти могут потребовать DDR5-5200 или ниже и ручной проверки стабильности.'),
('5.3 Оперативная память','Kingston FURY Beast 128 ГБ 4x32 DDR5-5200 - 235 718 ₽. RAM нужна для больших карт, rosbag, нескольких контейнеров, IDE, браузера и нескольких процессов Isaac Sim. 128 ГБ - практичный уровень для одной рабочей станции. 192/256 ГБ имеет смысл при нескольких экземплярах Isaac Sim, больших датасетах, локальном обучении и тяжёлых контейнерах; на AM5 это дороже и чувствительнее к частоте. После сборки проверить MemTest86 или OCCT, а EXPO включать только после базовой проверки.'),
('5.4 Видеокарта','MSI GeForce RTX 5090 Ventus 3X 32 ГБ - 571 119 ₽. 32 ГБ VRAM позволяют держать RTX-рендер, камеры, текстуры, несколько роботов и CUDA-модели с меньшим риском упора в память. Три вентилятора и большой радиатор нужны при длительной нагрузке. Более дешёвая RTX 4090 остаётся сильным вариантом, но у неё 24 ГБ и ниже запас для Isaac Sim/VR. Ограничения: высокая цена, 575 Вт TGP, переходник 12V-2x6 и необходимость хорошего воздушного потока.'),
('5.5 Накопители','Samsung 990 PRO 2 ТБ - 31 504 ₽ для Ubuntu, ROS2, IDE, Docker и приложений. WD Black SN850X 4 ТБ - 64 331 ₽ для rosbag, карт, моделей, видео и датасетов. Два SSD уменьшают конкуренцию последовательного I/O и упрощают резервное копирование. Один большой SSD дешевле по слотам, но при записи rosbag и обновлении ОС нагрузка смешивается. Оба диска имеют NVMe PCIe 4.0; их радиаторы должны быть установлены под штатными кожухами платы.'),
('5.6 Блок питания','DeepCool GamerStorm PQ1200G 1200 Вт ATX 3.1 - 13 570 ₽. RTX 5090 имеет заявленный TGP 575 Вт и кратковременные пики, поэтому нужен запас и нативный кабель 12V-2x6. 1200 Вт достаточно для одной карты и 9950X3D; 1600 Вт для этой конфигурации избыточен. Кабель 12V-2x6 вставить до упора и не изгибать вплотную к разъёму.'),
('5.7 Охлаждение и корпус','ARCTIC Liquid Freezer III Pro 360 - 11 349 ₽ поддерживает длительную нагрузку CPU. Lian Li Lancool III - 17 302 ₽ имеет простор для длинной видеокарты и уже четыре вентилятора 140 мм, поэтому дополнительные вентиляторы не обязательны. Добавлять два Noctua NF-A14 стоит только если длительный тест показывает высокую температуру GPU или накопителей.'),
('5.8 ИБП и кабели','APC BX1600MI-GR 1600 VA / 900 Вт - 30 323 ₽. Его задача - пережить краткий провал питания и дать время корректно остановить ПК; при полной нагрузке RTX 5090 автономность будет короткой. Робота, моторы и силовые приводы к этому ИБП не подключать. Резерв 3 000 ₽ покрывает Ethernet Cat6, кабель 12V-2x6 при необходимости, стяжки и крепления.')]
for h,txt in components:
    add_heading(doc,h,2); doc.add_paragraph(txt)

# 6
add_heading(doc,'6. Видеокарта для Isaac Sim и VR',1,'s6')
doc.add_paragraph('Isaac Sim использует видеопамять для RTX-рендеринга, трассировки лучей, PhysX, камер, текстур, материалов и геометрии сцен. При нескольких роботах или окружениях объём VRAM растёт раньше, чем вычислительная нагрузка CPU. Isaac Lab запускает поверх Isaac Sim сценарии обучения и пакетные окружения; при training дополнительно растут потребности в RAM, VRAM и пропускной способности памяти. Поэтому 32 ГБ у RTX 5090 дают запас, а не обязательный минимум.')
doc.add_paragraph('Официальная документация NVIDIA указывает для Isaac Sim минимум 32 ГБ RAM, 16 ГБ VRAM и уровень RTX 4080; для хорошего опыта - 64 ГБ RAM и RTX 5080; идеальный уровень в таблице - RTX PRO 6000 Blackwell 48 ГБ. Эти цифры относятся к Isaac Sim, а не к ROS2 или Gazebo. В нашем сценарии 128 ГБ RAM и 32 ГБ VRAM оставляют запас под камеры робота, VR и одновременные приложения.')
p=doc.add_paragraph('Отдельно полезно прочитать закрытый вопрос сообщества Isaac Lab ')
add_hyperlink(p,'GPU performance comparison for RL with Isaac Lab/Sim','https://github.com/isaac-sim/IsaacLab/issues/2761')
p.add_run(', открытый 23.06.2025. В нём обсуждаются правильные метрики для обучения: полный training FPS, объём VRAM и производительность связки симуляция плюс обучение. Это практический контекст, а не официальный бенчмарк: комментарии участников содержат оценки и ссылки, которые нужно проверять по первоисточникам. Для нашей сборки вывод осторожный: 32 ГБ VRAM помогают масштабировать параллельные окружения, но реальная скорость обучения зависит также от CPU, сценария, числа env и настроек PhysX.')
doc.add_paragraph('Для VR-телеприсутствия важны стабильный frametime, низкая задержка и резерв VRAM. Настройки нужно начинать с умеренного разрешения и одного вида робота, затем повышать качество после измерения задержки и температуры. RTX 5090 не заменяет оптимизацию сцен, ограничение FPS или правильную синхронизацию ROS2.')

# 7
add_heading(doc,'7. RTX 5090 и RTX 4090',1,'s7')
add_table(doc,['Параметр','RTX 5090','RTX 4090','Практический вывод'],[
('CUDA cores','21 760','16 384','5090 быстрее в тяжёлых CUDA и рендере, но ROS2 от этого не ускоряется автоматически'),
('VRAM','32 ГБ GDDR7','24 ГБ GDDR6X','5090 лучше для больших сцен, камер и нескольких окружений'),
('Шина памяти','512 бит','384 бит','У 5090 больше запас пропускной способности'),
('TGP','575 Вт','450 Вт','4090 проще охлаждать и питать'),
('Рекомендуемый БП NVIDIA','1000 Вт','850 Вт','Для сборки выбран запасной 1200 Вт'),
('Цена наблюдения','около 571 119 ₽','зависит от рынка','4090 имеет смысл, если 5090 выходит за бюджет или нужна меньшая тепловая нагрузка')],widths=[1.55,1.3,1.3,3.0],font_size=8.8)
doc.add_paragraph('Инженерная рекомендация: RTX 5090 выбирается из-за VRAM и запаса для Isaac Sim/VR. RTX 4090 остаётся хорошим компромиссом для одной сцены и большинства CUDA-задач. При ограниченном бюджете нельзя жертвовать качественным БП, охлаждением и SSD ради номинального прироста GPU.')

# 8
add_heading(doc,'8. Совместимость',1,'s8')
add_table(doc,['Проверка','Результат'],[
('CPU и плата','9950X3D использует AM5; X870E поддерживает Ryzen 9000 после актуального BIOS. Перед сборкой проверить версию BIOS на коробке или обновить через FlashBack.'),
('Память','DDR5 DIMM совместима с AM5; набор 4x32 может работать на 5200 MT/s, но не гарантируется профиль EXPO. Проверить QVL и стабильность на JEDEC/пониженной частоте.'),
('GPU и корпус','Lancool III рассчитан на крупные карты; перед заказом сверить фактическую длину MSI Ventus и место для кабеля 12V-2x6.'),
('SSD','Оба NVMe устанавливаются в M.2; использовать штатные радиаторы платы и оставить свободный поток воздуха.'),
('Питание','1200 Вт ATX 3.1 с нативным 12V-2x6 соответствует одной RTX 5090 и 9950X3D; не использовать сомнительные переходники.'),
('ОС и ПО','Ubuntu 24.04 LTS подходит для ROS2 Jazzy; Isaac Sim/Lab следует ставить по версии NVIDIA и требованиям конкретного релиза.'),
('Монитор','LG 32GR93U-B даёт 4K/144 Гц для Robot POV и VR-подготовки; для VR-очков всё равно потребуется отдельный совместимый шлем и DisplayPort/USB.')],widths=[1.75,5.4],font_size=8.9)

# 9
add_heading(doc,'9. Питание охлаждение и шум',1,'s9')
doc.add_paragraph('Расчётная пиковая нагрузка: GPU до 575 Вт, CPU до примерно 170 Вт в тяжёлой работе, остальная платформа и накопители около 100-150 Вт. 1200 Вт оставляет запас для кратковременных пиков. Настройка должна начинаться с заводских лимитов, без разгона. Нужны отдельные проверки: 30 минут GPU-нагрузки, CPU-нагрузки, совместная нагрузка, затем rosbag-запись и VR.')
doc.add_paragraph('Lancool III с четырьмя 140-мм вентиляторами обеспечивает базовый поток: передние вентиляторы на вдув, задний и верхние на выдув. Радиатор 360 мм разместить сверху, если фактическая совместимость позволяет; иначе использовать фронт с контролем направления потока. Цель для длительной работы - отсутствие троттлинга и стабильные температуры, а не минимальные цифры в простое.')
doc.add_paragraph('ИБП 900 Вт не должен использоваться как источник длительной полной мощности: при полной нагрузке запас по мощности мал. Для автономности больше нескольких минут потребуется отдельный ИБП большей мощности или серверный вариант, что в данный бюджет не включено.')

# 10
add_heading(doc,'10. Подключение Unitree R1 по Ethernet',1,'s10')
doc.add_paragraph('Рекомендуется выделить встроенный Ethernet-порт платы под private subnet робота и не смешивать его с NoMachine, VR-трафиком и обычным интернетом. Пример адресации из рабочего проекта: ПК 192.168.123.162/24, PC2 192.168.123.164. Конкретные адреса сверить с текущей конфигурацией R1 перед включением команд.')
doc.add_paragraph('ROS2-ноды управления, watchdog и аварийная остановка должны видеть робота через этот интерфейс. Интернет, обновления и удалённый рабочий стол лучше вести через Wi-Fi или второй сетевой интерфейс/USB-Ethernet, если он появится. Перед реальным роботом проверить только чтение, затем ограниченные команды, Deadman и E-stop. Физическое подключение робота и приводов требует отдельной проверки на месте и не заменяется этим отчётом.')

# 11
add_heading(doc,'11. Модернизация',1,'s11')
add_table(doc,['Когда','Модернизация','Зачем'],[
('Сейчас','Установить Ubuntu 24.04, ROS2 Jazzy, Gazebo и драйвер NVIDIA','Повторяемое программное окружение'),
('После стендовых тестов','Добавить 2x Noctua NF-A14 при высокой температуре','Снизить температуру SSD/GPU при непрерывной записи'),
('При нескольких Isaac Sim','Расширить RAM до 192/256 ГБ только после проверки QVL','Больше одновременных окружений и датасетов'),
('При росте rosbag','Добавить третий NVMe или NAS','Разделить рабочие данные и архив'),
('При обучении','Рассмотреть профессиональную GPU с 48 ГБ VRAM или вторую рабочую станцию','Большие модели и несколько независимых экспериментов'),
('При работе с реальным R1','Добавить физически раздельный сетевой интерфейс и аппаратный E-stop','Снизить риск смешения управления и служебного трафика')],widths=[1.55,3.25,2.35],font_size=8.7)

# 12
add_heading(doc,'12. Более дешёвая альтернатива',1,'s12')
doc.add_paragraph('Если RTX 5090 или память подорожают, рациональная альтернатива - Ryzen 9 9950X, RTX 4090 24 ГБ, 128 ГБ DDR5, те же два SSD, плата TUF X870E и БП 1200 Вт. Такая конфигурация обычно дешевле за счёт GPU и сохраняет сильную базу для ROS2, Gazebo, RViz, одной сцены Isaac Sim и VR. Ограничение - меньший запас VRAM и меньшая перспектива для нескольких окружений и тяжёлых текстур.')
doc.add_paragraph('Ещё дешевле можно оставить 2 ТБ системного SSD, временно отказаться от ИБП или выбрать монитор 4K/60 Гц. Эти шаги ухудшают удобство и устойчивость, поэтому для ежедневной работы с роботом они менее предпочтительны, чем экономия на дорогой материнской плате.')

# 13
add_heading(doc,'13. Итоговая рекомендация',1,'s13')
doc.add_paragraph('Покупать рекомендуется конфигурацию с Ryzen 9 9950X3D, RTX 5090 32 ГБ, 128 ГБ DDR5, двумя NVMe SSD, TUF X870E-PLUS WIFI7, 1200-Вт ATX 3.1, жидкостным охлаждением 360 мм, Lancool III, APC BX1600MI-GR и 4K-монитором. Это наиболее полный вариант в пределах 1 200 000 ₽ для разработки Unitree R1 и перехода от Gazebo к Isaac Sim, VR и реальному роботу.')
doc.add_paragraph('Три главные причины выбора: (1) 32 ГБ VRAM дают запас для Isaac Sim, камер, VR и CUDA; (2) 128 ГБ RAM и два SSD поддерживают одновременную симуляцию, IDE и запись rosbag; (3) плата, БП, корпус и private Ethernet сохраняют стабильность и расширяемость без переплаты за экстремальные функции.')
doc.add_paragraph('Ограничения: цены и наличие могут измениться; 4x32 ГБ требует проверки стабильности; 5090 потребляет много энергии и нуждается в аккуратной прокладке 12V-2x6; ИБП не питает робота; точная поддержка VR и Isaac Sim зависит от версий драйвера, Ubuntu и самого шлема. Перед оплатой сверить QVL памяти, BIOS, длину GPU, разъёмы корпуса и условия гарантии OEM-процессора.')
doc.add_paragraph('Порядок приёмки: собрать ПК без робота, обновить BIOS, проверить память и накопители, установить драйвер NVIDIA и ROS2/Gazebo, затем Isaac Sim/Lab, выполнить нагрузочные тесты, настроить отдельный Ethernet, и только после этого проводить ограниченные испытания R1 с Deadman и E-stop.')

# 14 sources
add_heading(doc,'14. Источники',1,'s14')
doc.add_paragraph('Источники открыты и проверены 28.09.2026. Цены приведены по независимым карточкам Яндекс Маркета; официальные страницы использованы для характеристик и совместимости; независимые обзоры - для практического сравнения производительности.')
source_groups=[
('Официальные требования и документация',[
('NVIDIA GeForce RTX 5090 - характеристики', 'https://www.nvidia.com/en-us/geforce/graphics-cards/50-series/rtx-5090/'),
('NVIDIA GeForce RTX 4090 - характеристики', 'https://www.nvidia.com/en-us/geforce/graphics-cards/40-series/rtx-4090/'),
('NVIDIA Isaac Sim - системные требования', 'https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html'),
('Isaac Lab - установка и требования', 'https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html'),
('Isaac Lab - benchmarks производительности RL', 'https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/performance_benchmarks.html'),
('ROS2 Jazzy - установка Ubuntu', 'https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html'),
('Gazebo Harmonic - установка Ubuntu', 'https://gazebosim.org/docs/harmonic/install_ubuntu/'),
('AMD Ryzen 9 9950X3D', 'https://www.amd.com/en/products/processors/desktops/ryzen/9000-series/amd-ryzen-9-9950x3d.html'),
('ASUS TUF Gaming X870E-PLUS WIFI7', 'https://www.asus.com/motherboards-components/motherboards/tuf-gaming/tuf-gaming-x870e-plus-wifi7/'),
('Samsung 990 PRO', 'https://semiconductor.samsung.com/consumer-storage/internal-ssd/990-pro/'),
('WD Black SN850X', 'https://www.sandisk.com/products/ssd/internal-ssd/wd-black-sn850x-nvme-ssd'),
('ARCTIC Liquid Freezer III Pro 360', 'https://www.arctic.de/en/Liquid-Freezer-III-Pro-360/ACFRE00180A'),
('Lian Li Lancool III', 'https://lian-li.com/product/lancool-iii/')]),
('Независимые тесты и обсуждения',[
('TechPowerUp - обзор GeForce RTX 5090 Founders Edition', 'https://www.techpowerup.com/review/nvidia-geforce-rtx-5090-founders-edition/'),
('Tom\'s Hardware - обзор GeForce RTX 5090', 'https://www.tomshardware.com/pc-components/gpus/nvidia-geforce-rtx-5090-review'),
('TechPowerUp - обзор Ryzen 9 9950X3D', 'https://www.techpowerup.com/review/amd-ryzen-9-9950x3d/'),
('ROS Discourse - обсуждения робототехников', 'https://discourse.ros.org/'),
('Isaac Lab issue 2761 - GPU performance comparison for RL with Isaac Lab/Sim', 'https://github.com/isaac-sim/IsaacLab/issues/2761'),
('Isaac Lab GitHub Issues - практические ограничения', 'https://github.com/isaac-sim/IsaacLab/issues/')]),
('Независимые ценовые наблюдения',[
('Ryzen 9 9950X3D OEM', 'https://market.yandex.ru/card/protsessor-amd-ryzen-9-9950x3d-am5-oem-100-000000719/6134854350'),
('ASUS TUF X870E-PLUS WIFI7', 'https://market.yandex.ru/card/asus-materinskaya-plata-tuf-gaming-x870e-plus-wifi7-am5-atx/4600397810'),
('Kingston FURY Beast 128 ГБ', 'https://market.yandex.ru/card/operativnaya-pamyat-kingston-fury-beast-kf552c40bwk4-128-ddr5---4x-32gb-5200mgts-dimm-white-ret/103165652107'),
('MSI RTX 5090 Ventus 3X', 'https://market.yandex.ru/card/videokarta-msi-gaming-rtx-5090-ventus-3x-32-gb-gddr7-512-bit/4352210330'),
('Samsung 990 PRO 2 ТБ', 'https://market.yandex.ru/card/ssd-nakopitel-samsung-990-pro-mz-v9p2t0bw-2tb-pcie-40-x4-m2-2280-nvme-m2/6263640315'),
('WD Black SN850X 4 ТБ', 'https://market.yandex.ru/card/tverdotelnyy-disk-4tb-wd-black-sn850x--m2-2280-pci-e-4x4-rw---73006300-mbs-tlc-3d-nand/5633650255'),
('DeepCool PQ1200G', 'https://market.yandex.ru/card/blok-pitaniya-deepcool-gamestorm-pq1200g-atx31-1200vt-black/5446874130'),
('ARCTIC Liquid Freezer III Pro 360', 'https://market.yandex.ru/card/sistema-okhlazhdeniya-zhidkostnaya-arctic-liquid-freezer-iii-pro-360/4588218604'),
('Lian Li Lancool III', 'https://market.yandex.ru/card/korpus-lian-li-lancool-iii-black-g99-lan3x00/103000764087'),
('APC BX1600MI-GR', 'https://market.yandex.ru/card/istochnik-bespereboynogo-pitaniya-apc-bx1600mi-gr-back-ups-1600va900w-230v-avr-4-schuko-sockets-usb/102206789404'),
('LG UltraGear 32GR93U-B', 'https://market.yandex.ru/card/315-monitor-lg-ultragear-32gr93u-b--169-3840x2160-140-ppi-144-gts-ips/103239556865')])]
for group,items in source_groups:
    add_heading(doc,group,2)
    for title,url in items: add_source_line(doc,title,url,'Источник')

# core properties
props=doc.core_properties; props.title='Рабочий ПК для Unitree R1'; props.subject='Конфигурация ПК для ROS2 Gazebo Isaac Sim VR и Unitree R1'; props.author='OpenAI Codex'; props.keywords='Unitree R1, ROS2, Gazebo, Isaac Sim, RTX 5090'
# keep headings and tables sane
for table in doc.tables:
    for row in table.rows:
        for cell in row.cells:
            for p in cell.paragraphs:
                p.paragraph_format.keep_together=True
# save
OUT.parent.mkdir(parents=True, exist_ok=True)
doc.save(OUT)
print(OUT)
