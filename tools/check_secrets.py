#!/usr/bin/env python3
"""Pre-commit secret check: refuses a commit whose staged changes contain a real secret.

Looks for the actual values in any local secrets.h (Wi-Fi, OTA and hotspot passwords), common
key/token formats, and files that should never be committed (secrets.h, keys, scratch logs).
Install once per clone:  git config core.hooksPath .githooks
"""
import glob, os, re, subprocess, sys

root = subprocess.run(['git', 'rev-parse', '--show-toplevel'], capture_output=True, text=True).stdout.strip()
os.chdir(root)

secrets = set()
for f in glob.glob('**/secrets.h', recursive=True):
    for _, v in re.findall(r'#define\s+(\w+)\s+"([^"]+)"', open(f, errors='ignore').read()):
        if len(v) >= 6 and v not in ('your-password', 'your-network-name', 'choose-a-password'):
            secrets.add(v)

patterns = {
    'private key': r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
    'GitHub token': r'gh[pousr]_[A-Za-z0-9]{30,}',
    'AWS key': r'AKIA[0-9A-Z]{16}',
    'API key (sk-...)': r'\bsk-(?:ant-)?[A-Za-z0-9_-]{20,}',
    'Google API key': r'AIza[0-9A-Za-z_-]{35}',
}
bad_files = [r'(^|/)secrets\.h$', r'\.(pem|key|p12|mobileprovision|der)$', r'_log\.txt$', r'(^|/)\.env$']

problems = []
staged = subprocess.run(['git', 'diff', '--cached', '--name-only', '--diff-filter=AM'], capture_output=True, text=True).stdout.split()
for f in staged:
    if any(re.search(p, f) for p in bad_files):
        problems.append(f'{f}: this kind of file should never be committed')
diff = subprocess.run(['git', 'diff', '--cached', '-U0', '--no-color'], capture_output=True, text=True, errors='ignore').stdout
cur = '?'
for line in diff.splitlines():
    if line.startswith('+++ b/'):
        cur = line[6:]
    elif line.startswith('+') and not line.startswith('+++'):
        for v in secrets:
            if v in line:
                problems.append(f'{cur}: contains a value from your secrets.h')
        for name, p in patterns.items():
            if re.search(p, line):
                problems.append(f'{cur}: looks like a {name}')

if problems:
    print('Commit blocked by the secret check:')
    for p in sorted(set(problems)):
        print('  - ' + p)
    print('Remove it from the commit (git restore --staged <file>), or fix the file, then commit again.')
    sys.exit(1)
