import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lucas-pc/ws_fruit_sorting/install/fruit_sorting_webots'
