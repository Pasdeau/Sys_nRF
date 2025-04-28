import re

with open('main.c', 'r', encoding='latin1') as f:
    content = f.read()

def replace_hex(match):
    hex_value = match.group(1)
    return bytes.fromhex(hex_value).decode('latin1')

new_content = re.sub(r'\\x([0-9A-Fa-f]{2})', replace_hex, content)

with open('main_converted.c', 'w', encoding='utf-8') as f:
    f.write(new_content)