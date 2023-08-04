
from lxml import etree

def add_gazebo_element_to_links(urdf_file):
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(urdf_file, parser)
    root = tree.getroot()

    for link in root.findall('link'):
        gazebo = etree.SubElement(link, 'gazebo')
        material = etree.SubElement(gazebo, 'material')
        material.text = 'Gazebo/Green'

    # Salvar o arquivo URDF modificado
    tree.write('modified_urdf.urdf', pretty_print=True)

add_gazebo_element_to_links('eboat4.urdf.xacro')
