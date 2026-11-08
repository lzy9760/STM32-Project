"""Compile the actual control source with fake MCU registers; never accesses hardware."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT/'tests'
BUILD = TESTS/'.build'


def scheduler():
    main = (ROOT/'Core/Src/main.c').read_text(encoding='utf-8')
    start = main.index('        static uint32_t spd_tick = 0U;')
    condition = re.search(r'if \(\(uint32_t\)\(spd_now - spd_tick\) >= (\d+)U\)',main[start:])
    if not condition or int(condition[1]) != 2:
        raise RuntimeError('Position scheduler test expects the checked-in 2 ms schedule')
    opening = main.index('{',start+condition.end())
    depth, end = 1, opening+1
    while depth:
        depth += (main[end]=='{')-(main[end]=='}')
        end += 1
    return ('static void foreground_position_service(void) {\n'
            'if (!foc_bumb && foc_aligned && !motorVars_M1.flagEnableAlignment) {\n'
            +main[start:end]+'\n}\n}\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc',default=shutil.which('gcc'),help='Host GCC executable')
    parser.add_argument('--check-snapshot',action='store_true',help='Also require unchanged release snapshot hashes')
    args = parser.parse_args()
    if not args.cc:
        parser.error('Install GCC or pass --cc /path/to/gcc')
    BUILD.mkdir(exist_ok=True)
    results = {}
    for name in ('speed','position'):
        case = BUILD/name
        case.mkdir(exist_ok=True)
        shutil.copyfile(TESTS/'stubs'/name/'main.h',case/'main.h')
        for header in ('foc.h','motor1_drive.h','encoder.h'):
            shutil.copyfile(ROOT/'Core/Inc'/header,case/header)
        (case/'debug.h').write_text('void Debug_Capture(float id,float iq);\n')
        if name == 'position':
            (case/'position_scheduler.h').write_text(scheduler())
        exe = case/(name+'.exe')
        subprocess.run([args.cc,'-std=c99','-O2','-Wall','-Wextra','-Werror',
                        '-Wno-unused-parameter','-I',str(case),str(TESTS/(name+'_host.c')),
                        '-lm','-o',str(exe)],check=True,timeout=60)
        run = subprocess.run([str(exe)],capture_output=True,text=True,timeout=30)
        print(run.stdout+run.stderr,end='')
        (case/'results.txt').write_text(run.stdout+run.stderr)
        run.check_returncode()
        results[name] = run.stdout.strip()
    project = ROOT/'MDK-ARM/XC5000_FOC.uvprojx'
    paths = [e.text for e in ET.parse(project).findall('.//FilePath')]
    missing = [p for p in paths if not (project.parent/p.replace('\\','/')).is_file()]
    if missing:
        raise RuntimeError('Missing Keil project sources: '+repr(missing))
    snapshot = json.loads((ROOT/'docs/source_snapshot.json').read_text())
    mismatch = [p for p,h in snapshot['core_sha256'].items()
                if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
    if mismatch and args.check_snapshot:
        raise RuntimeError('Snapshot differs: '+repr(mismatch))
    if mismatch:
        print('NOTE: edited Core files differ from the release snapshot: '+repr(mismatch))
    else:
        print('PASS: all 17 Core source hashes match the release snapshot.')
    print(f'PASS: all {len(paths)} Keil source entries resolve.')
    (BUILD/'summary.json').write_text(json.dumps({
        'hardware_access':False,'Keil_build':False,'results':results,
        'Keil_sources_resolved':len(paths),'core_snapshot_files':17},indent=2))


if __name__ == '__main__':
    main()
