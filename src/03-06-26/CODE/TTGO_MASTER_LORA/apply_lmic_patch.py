Import("env")
import os

old = 'activeChannels125khz > 6'
new = 'activeChannels125khz > 0'

def find_file(name):
    libdeps = os.path.join(env['PROJECT_DIR'], '.pio', 'libdeps')
    for root, dirs, files in os.walk(libdeps):
        if name in files:
            return os.path.join(root, name)
    return None

def patch(path, search, replace):
    with open(path, 'r') as f:
        content = f.read()
    if replace in content:
        return
    if search not in content:
        print("LMIC patch: pattern not found in", path)
        return
    with open(path, 'w') as f:
        f.write(content.replace(search, replace))
    print("LMIC patch: applied successfully")

single = any('SINGLE_CHANNEL_MODE' in str(f) for f in env.get('BUILD_FLAGS', []))
fp = find_file('lmic_us_like.c')

if fp:
    if single:
        patch(fp, old, new)
    else:
        patch(fp, new, old)
else:
    print("LMIC patch: lmic_us_like.c not found (library not yet downloaded?)")
