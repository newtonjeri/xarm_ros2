import socket
import netifaces
import re

def get_wired_ip_address():
    try:
        # Get all network interfaces
        interfaces = netifaces.interfaces()
        
        # Common wired interface names (add more if needed)
        wired_pattern = re.compile(r'^(eth|enp|ens|enx|eno)\d+')
        
        for interface in interfaces:
            if wired_pattern.match(interface):
                addrs = netifaces.ifaddresses(interface)
                if netifaces.AF_INET in addrs:
                    for addr_info in addrs[netifaces.AF_INET]:
                        if 'addr' in addr_info and not addr_info['addr'].startswith('127.'):
                            return addr_info['addr']
        return None
    except Exception as e:
        print(f"Error getting IP address: {e}")
        return None

# Alternative method using socket (gets primary IP, not specific to wired)
def get_ip_address_simple():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception as e:
        print(f"Error getting IP address: {e}")
        return None

if __name__ == "__main__":
    wired_ip = get_wired_ip_address()
    if wired_ip:
        print(f"Wired connection IP address: {wired_ip}")
    else:
        print("No wired connection found or couldn't determine IP")
        
    simple_ip = get_ip_address_simple()
    if simple_ip:
        print(f"Primary IP address: {simple_ip}")