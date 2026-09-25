"""
download_routes.py — Tai rlog ve may, dung cau truc thu muc ma lane_analysis.py can.

Xe chay ban fork qmpilot nen log KHONG nam tren server cua comma. Script nay lam
viec thang voi trang route cua qmpilot-server (trang co bang "segments:" liet ke
qlog.zst / rlog.zst cho tung segment).

CACH 1 — TU TRANG ROUTE (nen dung)
-----------------------------------
Mo trang route tren trinh duyet, copy URL tren thanh dia chi, roi:

    python download_routes.py aug31_dem="https://<host>/?route=8a83db1b6dedf381/0000000d--0e70addde5"

Script se doc trang, tim moi link rlog.zst co that va tai ve:

    routes/aug31_dem/segment_30/rlog.zst
    routes/aug31_dem/segment_31/rlog.zst
    ...

Nhieu route cung luc thi liet ke nhieu cap ten=URL.

CACH 2 — DO TRUC TIEP (khi khong dan duoc URL trang)
-----------------------------------------------------
    python download_routes.py --probe 47 \
        aug31_dem="https://<host>/8a83db1b6dedf381/0000000d--0e70addde5"

Script thu tung segment 0..46 xem co rlog.zst khong roi tai cai nao co.

CHI XEM CO GI, CHUA TAI
-----------------------
    python download_routes.py --check aug31_dem="<url>"

In ra segment nao da co rlog, segment nao moi chi co qlog. Dung cai nay TRUOC
khi tai de biet route co dang tai khong.

TUY CHON
--------
    --out THUMUC      noi luu (mac dinh: routes)
    --jobs 6          so file tai song song
    --cookie "..."    neu trang doi dang nhap, mo DevTools > Network > copy header Cookie
    --header "K: V"   them header bat ky (dung nhieu lan duoc)

Chay lai bao nhieu lan cung duoc: file nao tai du roi thi bo qua. Mang dut giua
chung cu chay lai lenh cu.
"""

import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed

TIMEOUT = 60
HEADERS = {"User-Agent": "Mozilla/5.0 (lane-analysis downloader)"}


def http(url, method="GET"):
    req = urllib.request.Request(url, headers=HEADERS, method=method)
    return urllib.request.urlopen(req, timeout=TIMEOUT)


# ── tim link rlog ─────────────────────────────────────────────────────────
def links_from_page(page_url):
    """Doc trang route, tra ve {so_segment: url_rlog}."""
    try:
        with http(page_url) as r:
            html = r.read().decode("utf-8", "replace")
            base = r.geturl()
    except urllib.error.HTTPError as e:
        sys.exit(f"Khong mo duoc trang ({e.code}). Neu trang doi dang nhap, them --cookie \"...\"")
    except urllib.error.URLError as e:
        sys.exit(f"Khong ket noi duoc: {e.reason}")

    out = {}
    for href in re.findall(r'href=["\']([^"\']+)["\']', html):
        if "rlog" not in href or "qlog" in href:
            continue
        full = urllib.parse.urljoin(base, href)
        # .../<route>/<so_segment>/rlog.zst  hoac  .../<route>--<so_segment>/rlog...
        m = re.search(r"/(\d+)/rlog[^/]*$", full) or re.search(r"--(\d+)/rlog[^/]*$", full)
        if m:
            out[int(m.group(1))] = full
    if not out:
        n_q = len(re.findall(r"qlog", html))
        print(f"  Trang nay khong co link rlog nao (tim thay {n_q} cho nhac qlog).")
    return out


def links_by_probe(base_url, n_seg):
    """Thu tung segment xem file rlog co ton tai khong."""
    base = base_url.rstrip("/")
    found = {}

    def check(i):
        for name in ("rlog.zst", "rlog.bz2", "rlog"):
            for u in (f"{base}/{i}/{name}", f"{base}--{i}/{name}"):
                try:
                    with http(u, "HEAD") as r:
                        if r.status == 200:
                            return i, u
                except Exception:
                    continue
        return i, None

    with ThreadPoolExecutor(max_workers=8) as ex:
        for i, u in ex.map(check, range(n_seg)):
            if u:
                found[i] = u
    return found


# ── tai ───────────────────────────────────────────────────────────────────
def fetch(url, dest, retries=3):
    if os.path.exists(dest) and os.path.getsize(dest) > 1024:
        return True, "co san"
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    tmp = dest + ".part"
    for k in range(retries):
        try:
            with http(url) as r, open(tmp, "wb") as f:
                while True:
                    chunk = r.read(1 << 20)
                    if not chunk:
                        break
                    f.write(chunk)
            if os.path.getsize(tmp) < 1024:
                raise IOError("file rong")
            os.replace(tmp, dest)
            return True, "tai xong"
        except Exception as e:
            if k == retries - 1:
                if os.path.exists(tmp):
                    os.remove(tmp)
                return False, f"{type(e).__name__}: {e}"
            time.sleep(2 * (k + 1))


def summary(label, links, n_probe=None):
    if not links:
        print(f"  {label}: CHUA CO segment nao co rlog")
        return
    seg = sorted(links)
    runs, start = [], seg[0]
    for a, b in zip(seg, seg[1:] + [None]):
        if b != (a + 1):
            runs.append(f"{start}" if start == a else f"{start}-{a}")
            start = b
    total = f"/{n_probe}" if n_probe else ""
    print(f"  {label}: co rlog o {len(seg)}{total} segment -> {', '.join(runs)}")


def run(label, url, outdir, jobs, probe, check_only):
    print(f"\n=== {label} ===")
    links = links_by_probe(url, probe) if probe else links_from_page(url)
    summary(label, links, probe)
    if check_only or not links:
        return 0

    jobs_list = []
    for i, u in sorted(links.items()):
        ext = ".bz2" if u.split("?")[0].endswith(".bz2") else (
            "" if u.split("?")[0].endswith("rlog") else ".zst")
        jobs_list.append((u, os.path.join(outdir, label, f"segment_{i}", "rlog" + ext)))

    done = 0
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = {ex.submit(fetch, u, d): (u, d) for u, d in jobs_list}
        for n, f in enumerate(as_completed(futs), 1):
            ok, msg = f.result()
            if ok:
                done += 1
            else:
                print(f"  LOI {os.path.basename(os.path.dirname(futs[f][1]))}: {msg}")
            if n % 10 == 0 or n == len(jobs_list):
                print(f"  [{n}/{len(jobs_list)}] xong {done}")
    return done


def main(argv):
    if not argv or "--help" in argv or "-h" in argv:
        print(__doc__)
        return

    outdir, jobs, probe, check_only, pairs = "routes", 6, None, False, []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--out":
            outdir = argv[i + 1]; i += 2
        elif a == "--jobs":
            jobs = int(argv[i + 1]); i += 2
        elif a == "--probe":
            probe = int(argv[i + 1]); i += 2
        elif a == "--check":
            check_only = True; i += 1
        elif a == "--cookie":
            HEADERS["Cookie"] = argv[i + 1]; i += 2
        elif a == "--header":
            k, v = argv[i + 1].split(":", 1)
            HEADERS[k.strip()] = v.strip(); i += 2
        elif "=" in a and "://" in a:
            label, url = a.split("=", 1)
            pairs.append((label, url)); i += 1
        elif "://" in a:
            pairs.append((f"route{len(pairs)+1}", a)); i += 1
        else:
            sys.exit(f"Khong hieu tham so: {a}\nChay --help de xem cach dung.")

    if not pairs:
        print(__doc__)
        return

    t0 = time.time()
    total = sum(run(l, u, outdir, jobs, probe, check_only) for l, u in pairs)
    if check_only:
        print("\nDang o che do --check, chua tai gi ca.")
        return
    print(f"\nXong: {total} file trong {(time.time()-t0)/60:.1f} phut -> {os.path.abspath(outdir)}")
    if total:
        print("\nChay phan tich:")
        print("  python lane_analysis.py " + " ".join(os.path.join(outdir, l) for l, _ in pairs))


if __name__ == "__main__":
    main(sys.argv[1:])
