import os
import xml.etree.ElementTree as ET

def find_ros2_packages(root_dir):
    packages = []

    for dirpath, _, filenames in os.walk(root_dir):
        package_info = {}
        if 'package.xml' in filenames and 'CMakeLists.txt' in filenames:
            
            # print(f"Found package.xml in {dirpath}")
            
            package_info['root'] = dirpath
            package_info['dependencies'] = {'build': [], 'exec': [], 'test': []}

            # Parse package.xml
            package_xml_path = os.path.join(dirpath, 'package.xml')
            # input(os.path.isfile(package_xml_path))
            tree = ET.parse(package_xml_path)
            root = tree.getroot()

            # Extract package name
            name_tag = root.find('name')
            package_info['name'] = name_tag.text if name_tag is not None else 'Unknown'

            # Extract dependencies
            for dep_tag in root.findall('depend'):
                package_info['dependencies']['exec'].append(dep_tag.text)
            for build_dep_tag in root.findall('build_depend'):
                package_info['dependencies']['build'].append(build_dep_tag.text)
            for test_dep_tag in root.findall('test_depend'):
                package_info['dependencies']['test'].append(test_dep_tag.text)

            cmake_path = os.path.join(dirpath, 'CMakeLists.txt')
            with open(cmake_path, 'r') as cmake_file:
                cmake_content = cmake_file.read()
                if 'ament_' in cmake_content:
                    packages.append(package_info)

    return packages

def generate_markdown_table(packages):
    table = "| Root Path | Package Name | Build Dependencies | Exec Dependencies | Test Dependencies |\n"
    table += "|-----------|--------------|--------------------|-------------------|-------------------|\n"
    for pkg in packages:
        table += f"| {pkg['root']} | {pkg['name']} | {', '.join(pkg['dependencies']['build'])} | {', '.join(pkg['dependencies']['exec'])} | {', '.join(pkg['dependencies']['test'])} |\n"
    return table

if __name__ == "__main__":
    root_directory = "."  # Change this to your repo's root directory
    ros2_packages = find_ros2_packages(root_directory)
    markdown_table = generate_markdown_table(ros2_packages)
    with open('ros2_packages_scan.md', 'w') as f:
        f.write(markdown_table)
    
    print("ROS2 packages scan completed. Results saved to 'ros2_packages_scan.md'.")
    