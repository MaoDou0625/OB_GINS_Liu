# -*- coding: utf-8 -*-
import os, shutil
src=r'D:/Code/OB_GINS/dataset/ADIS16465.txt'
bak=r'D:/Code/OB_GINS/dataset/ADIS16465_withodo.txt'
tmp=r'D:/Code/OB_GINS/dataset/ADIS16465.txt.tmp'
# backup once if not exists
if not os.path.exists(bak):
    shutil.copy2(src, bak)
# rewrite first 7 columns
n_in=0; n_out=0
with open(src,'r') as fin, open(tmp,'w') as fout:
    for line in fin:
        s=line.strip().split()
        if not s:
            continue
        n_in+=1
        # keep at most 7 columns
        keep=s[:7] if len(s)>=7 else s
        fout.write(' '.join(keep)+'\n')
        n_out+=1
os.replace(tmp, src)
print('rows_in', n_in, 'rows_out', n_out, '-> trimmed to 7 cols')
