"""Make the app icon: square 1024 master from the artwork, then every iOS and Android size."""
import json, os, sys
from PIL import Image, ImageChops

SRC = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/Downloads/file.png')
FILL = float(sys.argv[2]) if len(sys.argv) > 2 else 0.88  # share of the icon the artwork fills
APP = os.path.expanduser('~/tankbot-work/tankbot-platform/app/tankbot_brain')
OUT = APP + '/assets/icon'
os.makedirs(OUT, exist_ok=True)

im = Image.open(SRC).convert('RGB')
w, h = im.size
# background colour: the median of the four corners
corners = [im.getpixel(p) for p in [(2, 2), (w - 3, 2), (2, h - 3), (w - 3, h - 3)]]
bg = tuple(sorted(c[i] for c in corners)[1] for i in range(3))
# bounding box of everything that isn't background
diff = ImageChops.difference(im, Image.new('RGB', im.size, bg)).convert('L').point(lambda v: 255 if v > 18 else 0)
box = diff.getbbox()
art = im.crop(box)
aw, ah = art.size
print('source %dx%d, background %s, artwork box %s (%dx%d)' % (w, h, bg, box, aw, ah))

# square master: artwork scaled to fill FILL of the icon (88% keeps the treads clear of iOS's rounded corners)
S = 1024
scale = (S * FILL) / max(aw, ah)
art = art.resize((round(aw * scale), round(ah * scale)), Image.LANCZOS)
master = Image.new('RGB', (S, S), bg)
master.paste(art, ((S - art.width) // 2, (S - art.height) // 2))
master.save(OUT + '/app_icon_1024.png')
print('master 1024x1024 (artwork scaled x%.2f)' % scale)

# iOS: every image listed in the asset catalog
ios = APP + '/ios/Runner/Assets.xcassets/AppIcon.appiconset'
contents = json.load(open(ios + '/Contents.json'))
n = 0
for img in contents['images']:
    fn = img.get('filename')
    if not fn:
        continue
    pts = float(img['size'].split('x')[0])
    px = round(pts * float(img['scale'].rstrip('x')))
    master.resize((px, px), Image.LANCZOS).save(os.path.join(ios, fn))  # RGB: no alpha, as iOS requires
    n += 1
print('iOS: %d icon images written' % n)

# Android launcher icons (if the android folder exists)
sizes = {'mdpi': 48, 'hdpi': 72, 'xhdpi': 96, 'xxhdpi': 144, 'xxxhdpi': 192}
res = APP + '/android/app/src/main/res'
m = 0
for d, px in sizes.items():
    f = '%s/mipmap-%s/ic_launcher.png' % (res, d)
    if os.path.exists(os.path.dirname(f)):
        master.resize((px, px), Image.LANCZOS).save(f)
        m += 1
print('Android: %d icon images written' % m)
