import socket
import netifaces
import re
import netifaces

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
# def get_ip_address_simple():
#     try:
#         s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         s.connect(("8.8.8.8", 80))
#         ip = s.getsockname()
#         s.close()
#         return ip
#     except Exception as e:
#         print(f"Error getting IP address: {e}")
#         return None


def list_network_interfaces():
    try:
        interfaces = netifaces.interfaces()
        for interface in interfaces:
            print(f"Interface: {interface}")
            addrs = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addrs:
                for addr_info in addrs[netifaces.AF_INET]:
                    print(f"  IPv4 Address: {addr_info['addr']}")
                    if 'netmask' in addr_info:
                        print(f"  Netmask: {addr_info['netmask']}")
                    if 'broadcast' in addr_info:
                        print(f"  Broadcast: {addr_info['broadcast']}")
            if netifaces.AF_INET6 in addrs:
                for addr_info in addrs[netifaces.AF_INET6]:
                    print(f"  IPv6 Address: {addr_info['addr']}")
            print("-" * 30)
    except Exception as e:
        print(f"Error listing network interfaces: {e}")


def list_ip_addresses():
    try:
        hostname = socket.gethostname()
        ip_list = socket.gethostbyname_ex(hostname)[2]
        print("Available IP Addresses:")
        for ip in ip_list:
            print(f"- {ip}")
    except Exception as e:
        print(f"Error listing IP addresses: {e}")

def get_ip_address_simple():
    """
    Lists all available network interfaces and their IP addresses,
    then allows the user to select which one to use.
    """
    import netifaces
    
    try:
        print("\n" + "="*50)
        print("🌐 AVAILABLE NETWORK INTERFACES:")
        print("="*50)
        
        interfaces = netifaces.interfaces()
        valid_interfaces = []
        
        for i, interface in enumerate(interfaces):
            try:
                # Get IPv4 addresses for this interface
                addrs = netifaces.ifaddresses(interface)
                if netifaces.AF_INET in addrs:
                    ipv4_info = addrs[netifaces.AF_INET][0]
                    ip_addr = ipv4_info['addr']
                    netmask = ipv4_info.get('netmask', 'N/A')
                    
                    # Skip loopback addresses
                    if not ip_addr.startswith('127.'):
                        valid_interfaces.append((interface, ip_addr, netmask))
                        print(f"{len(valid_interfaces)}. {interface:12} - {ip_addr:15} (Netmask: {netmask})")
                        
            except (KeyError, IndexError):
                # Skip interfaces without IPv4 addresses
                continue
        
        if not valid_interfaces:
            print("⚠️  No valid network interfaces found!")
            # Fallback to original method
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        
        print("="*50)
        
        # Let user choose interface
        while True:
            try:
                choice = input(f"Select interface (1-{len(valid_interfaces)}) or press Enter for auto-detect: ").strip()
                
                if choice == "":
                    # Auto-detect using original method
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.connect(("8.8.8.8", 80))
                    ip = s.getsockname()[0]
                    s.close()
                    print(f"🔍 Auto-detected IP: {ip}")
                    return ip
                
                choice_num = int(choice)
                if 1 <= choice_num <= len(valid_interfaces):
                    selected_interface, selected_ip, selected_netmask = valid_interfaces[choice_num - 1]
                    print(f"✅ Selected: {selected_interface} - {selected_ip}")
                    return selected_ip
                else:
                    print(f"❌ Invalid choice. Please enter a number between 1 and {len(valid_interfaces)}")
                    
            except ValueError:
                print("❌ Invalid input. Please enter a number or press Enter for auto-detect.")
            except KeyboardInterrupt:
                print("\n🚪 Operation cancelled by user")
                return None
                
    except ImportError:
        print("⚠️  netifaces module not found. Using fallback method...")
        print("   Install with: pip install netifaces")
        # Fallback to original method
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception as e:
            print(f"Error getting IP address: {e}")
            return None
    except Exception as e:
        print(f"Error listing network interfaces: {e}")
        # Fallback to original method
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception as fallback_e:
            print(f"Error getting IP address: {fallback_e}")
            return None

if __name__ == "__main__":
    # wired_ip = get_wired_ip_address()
    # if wired_ip:
    #     print(f"Wired connection IP address: {wired_ip}")
    # else:
    #     print("No wired connection found or couldn't determine IP")
        
    # simple_ip = get_ip_address_simple()
    # if simple_ip:
    #     print(f"Primary IP address: {simple_ip}")

    # list_network_interfaces()
    # list_ip_addresses()
    get_ip_address_simple()