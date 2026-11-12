#!/usr/bin/env python3
"""Download the official vocabulary archive and verify its locked SHA256."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import urllib.request


def prepare(destination):
    lock = json.loads((Path(__file__).resolve().parents[1] / 'dependencies.lock.json').read_text())
    spec = lock['resources']['vocabularies']
    destination.mkdir(parents=True, exist_ok=True)
    archive = destination / 'vocabularies.tar.gz'
    if not archive.exists():
        temporary = destination / 'vocabularies.tar.gz.part'
        urllib.request.urlretrieve(spec['url'], temporary)
        temporary.rename(archive)
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    if digest != spec['sha256']:
        raise RuntimeError(f'Vocabulary SHA256 mismatch: {digest}')
    with tarfile.open(archive) as bundle:
        if sorted(bundle.getnames()) != sorted(spec['files']):
            raise RuntimeError('Unexpected vocabulary archive members')
        for member in bundle.getmembers():
            if not member.isfile() or Path(member.name).name != member.name:
                raise RuntimeError('Unsafe vocabulary archive member')
        bundle.extractall(destination / 'vocabularies', filter='data')
    print(destination / 'vocabularies')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    prepare(parser.parse_args().directory.resolve())
