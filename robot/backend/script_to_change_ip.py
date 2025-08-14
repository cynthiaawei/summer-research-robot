
import os
import glob

old_ip = "172.31.176.1"
new_ip = input("Enter new IP address: ")

# Find all Python and XML files
for pattern in ["**/*.py", "**/*.xml", "**/*.yaml"]:
    for filepath in glob.glob(f"/home/{os.getenv('USER')}/summer-research-robot/{pattern}", recursive=True):
        try:
            with open(filepath, 'r') as f:
                content = f.read()
            
            if old_ip in content:
                print(f"Replacing in: {filepath}")
                content = content.replace(old_ip, new_ip)
                with open(filepath, 'w') as f:
                    f.write(content)
        except Exception as e:
            print(f"Error with {filepath}: {e}")

print("Done!")
