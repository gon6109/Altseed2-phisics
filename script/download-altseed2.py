import requests
import sys
import os
import zipfile
from dotenv import load_dotenv


def download(url):
    binary = requests.get(
        url, headers={'Authorization': f'token {os.environ.get("GITHUB_ACCESS_TOKEN")}'})
    with open('../lib/altseed2.zip', mode='wb') as f:
        for chunk in binary.iter_content(chunk_size=1024):
            if chunk:
                f.write(chunk)
                f.flush()

    with zipfile.ZipFile('../lib/altseed2.zip') as zip:
        zip.extractall('../lib')


os.chdir(os.path.dirname(__file__))
load_dotenv("../.env", verbose=True)

commit_hash = None
if len(sys.argv) > 1:
    commit_hash = sys.argv[1]
    print(commit_hash)

res = requests.get(
    "https://api.github.com/repos/altseed/Altseed2-csharp/actions/artifacts?per_page=1000")

if commit_hash:
    for artifact in res.json()['artifacts']:
        if artifact['name'] == 'Altseed2-' + commit_hash:
            print(f'download: {artifact["name"]}')
            download(artifact['archive_download_url'])
            sys.exit()

for artifact in res.json()['artifacts']:
    if 'Altseed2-' in artifact['name']:
        print(f'download: {artifact["name"]}')
        download(artifact['archive_download_url'])
        sys.exit()
