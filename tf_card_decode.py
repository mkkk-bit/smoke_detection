#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import struct, sys, os, argparse

LOG_BASE          = 8      # 日志起始扇区
SECTORS_PER_ALARM = 121    # 1 日志 + 120 缩略图
RING_SECTORS      = 1936   # 环形扇区数 (16 次告警 x 121)
THUMB_W           = 256    # 缩略图宽 (1280/5)
THUMB_H           = 120    # 缩略图高 (720/6)
SECTOR            = 512
MAGIC             = b'\xa5\xa5\xa5\xa5'


def parse_log(b):
    """解析 512B 日志扇区, 魔数不匹配返回 None"""
    if b[:4] != MAGIC:
        return None
    frame     = struct.unpack_from('<I', b, 4)[0]
    fire_area = b[8] | (b[9] << 8) | ((b[10] & 0x0F) << 16)
    cig_count = b[12] | (b[13] << 8) | ((b[14] & 0x0F) << 16)
    fx0 = b[16] | ((b[17] & 0x07) << 8)
    fx1 = b[18] | ((b[19] & 0x07) << 8)
    fy0 = b[20] | ((b[21] & 0x03) << 8)
    fy1 = b[22] | ((b[23] & 0x03) << 8)
    cx0 = b[24] | ((b[25] & 0x07) << 8)
    cx1 = b[26] | ((b[27] & 0x07) << 8)
    cy0 = b[28] | ((b[29] & 0x03) << 8)
    cy1 = b[30] | ((b[31] & 0x03) << 8)
    fv  = b[32] & 1          # bit0 = fire (与 snap_store.v 编码一致)
    cv  = (b[32] >> 1) & 1   # bit1 = cig
    ts  = struct.unpack_from('<I', b, 34)[0]
    return dict(frame=frame, fire_area=fire_area, cig_count=cig_count,
                fx0=fx0, fx1=fx1, fy0=fy0, fy1=fy1,
                cx0=cx0, cx1=cx1, cy0=cy0, cy1=cy1,
                fire_v=fv, cig_v=cv, ts=ts)


def rgb565_to_rgb888(data):
    """RGB565(小端: 低字节在前) -> RGB888 (位复制展开)"""
    n = len(data) // 2
    out = bytearray(n * 3)
    for i in range(n):
        lo = data[2 * i]
        hi = data[2 * i + 1]
        r5 = (hi >> 3) & 0x1F
        g6 = ((hi & 0x07) << 3) | ((lo >> 5) & 0x07)
        b5 = lo & 0x1F
        out[3 * i]     = (r5 << 3) | (r5 >> 2)          # 5bit -> 8bit
        out[3 * i + 1] = (g6 << 2) | (g6 >> 4)          # 6bit -> 8bit
        out[3 * i + 2] = (b5 << 3) | (b5 >> 2)          # 5bit -> 8bit
    return bytes(out)


def decode_thumb(data, scale):
    """RGB565 缩略图放大 -> (w,h,RGB bytes)。PIL 双线性, 保持 16:9; 无 PIL 回退最近邻"""
    rgb = rgb565_to_rgb888(data)
    out_w = THUMB_W * scale
    out_h = out_w * 9 // 16              # 16:9 (源 1280x720)
    try:
        from PIL import Image
        im = Image.frombytes('RGB', (THUMB_W, THUMB_H), rgb)
        im = im.resize((out_w, out_h), Image.BILINEAR)
        return out_w, out_h, im.tobytes()
    except ImportError:
        out = bytearray(out_w * out_h * 3)
        for y in range(out_h):
            sy = y * THUMB_H // out_h
            for x in range(out_w):
                sx = x * THUMB_W // out_w
                src = (sy * THUMB_W + sx) * 3
                dst = (y * out_w + x) * 3
                out[dst:dst + 3] = rgb[src:src + 3]
        return out_w, out_h, bytes(out)


def write_ppm(path, w, h, data):
    with open(path, 'wb') as f:
        f.write(('P6\n%d %d\n255\n' % (w, h)).encode())
        f.write(data)


def write_png(path, w, h, data):
    try:
        from PIL import Image
        Image.frombytes('RGB', (w, h), data).save(path)
        return True
    except ImportError:
        return False


def read_image(args):
    if args.image:
        with open(args.image, 'rb') as f:
            return f.read()
    else:
        drive = '\\\\.\\PhysicalDrive%d' % args.drive
        print('读取 %s ... (若提示权限错误, 请用管理员运行, 或先用 dd/HxD 导出镜像)' % drive)
        with open(drive, 'rb') as f:
            return f.read((LOG_BASE + RING_SECTORS + 16) * SECTOR)


def main():
    ap = argparse.ArgumentParser(description='TF卡告警快照解码器 (RGB565彩色)')
    ap.add_argument('--image', help='SD卡镜像文件(.img/.bin)')
    ap.add_argument('--drive', type=int, help='直接读 PhysicalDriveN (需管理员)')
    ap.add_argument('--scale', type=int, default=5, help='宽放大倍数(默认5 -> 1280x720)')
    ap.add_argument('--outdir', default='tf_out', help='输出目录(默认 tf_out)')
    args = ap.parse_args()

    if not args.image and args.drive is None:
        ap.error('必须指定 --image 或 --drive')

    img = read_image(args)
    total_sectors = len(img) // SECTOR
    print('读取 %d 字节 = %d 扇区' % (len(img), total_sectors))
    if total_sectors < LOG_BASE + RING_SECTORS:
        print('警告: 镜像不足 %d 扇区, 可能不是完整 SD 卡转储' % (LOG_BASE + RING_SECTORS))

    os.makedirs(args.outdir, exist_ok=True)
    n_found = 0

    for i in range(RING_SECTORS // SECTORS_PER_ALARM):
        sec = LOG_BASE + i * SECTORS_PER_ALARM
        if (sec + SECTORS_PER_ALARM) * SECTOR > len(img):
            break
        log = parse_log(img[sec * SECTOR:(sec + 1) * SECTOR])
        if log is None:
            continue

        raw = img[(sec + 1) * SECTOR:(sec + SECTORS_PER_ALARM) * SECTOR][:THUMB_W * THUMB_H * 2]
        if len(raw) < THUMB_W * THUMB_H * 2:
            print('[槽位%02d] 缩略图数据不足, 跳过' % i)
            continue
        w, h, up = decode_thumb(raw, args.scale)
        base = os.path.join(args.outdir, 'thumb_%02d' % i)
        png_ok = write_png(base + '.png', w, h, up)
        if not png_ok:
            write_ppm(base + '.ppm', w, h, up)

        typ = ('明火' if log['fire_v'] else '') + ('+' if log['fire_v'] and log['cig_v'] else '') + ('烟头' if log['cig_v'] else '')
        if not typ:
            typ = '无'
        print('[槽位%02d] 扇区%d 帧号=%d 类型=%s 明火面积=%d 烟头计数=%d 明火框=(%d,%d)-(%d,%d) 烟头框=(%d,%d)-(%d,%d)' % (
            i, sec, log['frame'], typ, log['fire_area'], log['cig_count'],
            log['fx0'], log['fy0'], log['fx1'], log['fy1'],
            log['cx0'], log['cy0'], log['cx1'], log['cy1']))
        n_found += 1

    print('\n共解析 %d 条有效告警, 缩略图已存到 %s/' % (n_found, args.outdir))
    if n_found == 0:
        print('未找到有效数据。可能原因: ①还没触发过告警 ②读错了盘 ③扇区被覆盖 ④旧固件数据格式不符')


if __name__ == '__main__':
    main()
