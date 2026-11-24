"""alertmanager.yml dosyasina Telegram mesaj sablonunu ekler.

Kullanim (monitoring klasorunden):  python3 telegram_sablonu_kur.py

- Token ve chat_id'ye dokunmaz, ekrana da yazmaz.
- Eklenen satirlar: en uste 'templates:' blogu, telegram_configs altina
  'parse_mode: HTML' ve 'message: ...'.
- Ikinci kez calistirilirsa hicbir sey degistirmez.
- Degistirmeden once yedegi ev klasorune koyar (~/alertmanager.yml.yedek),
  depo icine koymaz: yedekte token var, yanlislikla commit edilmesin.
"""
import os
import re
import shutil
import sys

YOL = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'alertmanager.yml')
YEDEK = os.path.expanduser('~/alertmanager.yml.yedek')
ISARET = 'iha.telegram'


def main():
    if not os.path.exists(YOL):
        sys.exit(f'Bulunamadi: {YOL}')
    metin = open(YOL, encoding='utf-8').read()
    if ISARET in metin:
        print('Sablon zaten ekli, bir sey degismedi.')
        return
    satirlar = metin.splitlines()

    # telegram_configs altindaki 'send_resolved' satirini bul, ayni girintiyle iki satir ekle
    yer = next((i for i, s in enumerate(satirlar) if re.match(r'\s*send_resolved\s*:', s)), None)
    if yer is None:
        # send_resolved yoksa bot_token satirinin girintisini kullan
        yer = next((i for i, s in enumerate(satirlar) if re.match(r'\s*-?\s*bot_token\s*:', s)), None)
        if yer is None:
            sys.exit('telegram_configs bulunamadi; dosya beklenen yapida degil, elle bakmak gerekiyor.')
        g = re.match(r'(\s*)-?\s*', satirlar[yer]).end()
    else:
        g = len(satirlar[yer]) - len(satirlar[yer].lstrip())
    girinti = ' ' * g
    satirlar[yer + 1:yer + 1] = [
        f'{girinti}parse_mode: HTML',
        f"{girinti}message: '{{{{ template \"{ISARET}\" . }}}}'",
    ]

    # 'route:' ile ayni girintide en uste templates blogu
    kok = next((s for s in satirlar if re.match(r'\s*route\s*:', s)), 'route:')
    kg = ' ' * (len(kok) - len(kok.lstrip()))
    satirlar[0:0] = [f'{kg}templates:', f'{kg}  - /etc/alertmanager/telegram.tmpl']

    shutil.copy2(YOL, YEDEK)
    open(YOL, 'w', encoding='utf-8').write('\n'.join(satirlar) + '\n')
    print('Sablon eklendi. Eski dosyanin yedegi:', YEDEK)


if __name__ == '__main__':
    main()
