#!/usr/bin/python

#Audio Tools, a module and set of tools for manipulating audio data
#Copyright (C) 2007-2013  Brian Langenberger

#This program is free software; you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation; either version 2 of the License, or
#(at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.

#You should have received a copy of the GNU General Public License
#along with this program; if not, write to the Free Software
#Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

from audiotools.bitstream import BitstreamWriter
from audiotools import BufferedPCMReader


def div_ceil(n, d):
    """returns the ceiling of n divided by d as an int"""

    return n // d + (1 if ((n % d) != 0) else 0)


def encode_tta(file, pcmreader):
    """given a file object and buffered PCMReader,
    writes TTA frames to the writer
    and returns a list of TTA frame lengths, in bytes"""

    writer = BitstreamWriter(file, True)
    block_size = (pcmreader.sample_rate * 256) / 245
    frame_sizes = []

    #encode FrameLists from PCMReader to temporary space
    framelist = pcmreader.read(block_size)
    while (len(framelist) > 0):
        frame_sizes.append(encode_tta_frame(writer,
                                            pcmreader.bits_per_sample,
                                            framelist))
        framelist = pcmreader.read(block_size)

    writer.flush()

    return frame_sizes


def encode_tta_frame(writer, bits_per_sample, framelist):
    frame_crc = CRC32()
    counter = Counter()
    writer.add_callback(counter.update)
    writer.add_callback(frame_crc.update)

    #correlate channels
    if (framelist.channels == 1):
        correlated = [list(framelist.channel(0))]
    else:
        correlated = correlate_channels([list(framelist.channel(i))
                                         for i in xrange(framelist.channels)])

    residuals = []
    for correlated_ch in correlated:
        #run fixed order prediction
        predicted = fixed_predictor(bits_per_sample, correlated_ch)

        #run hybrid filter
        residuals.append(tta_filter(bits_per_sample, predicted))

    #setup Rice parameters for each channel
    k0 = [10] * framelist.channels
    k1 = [10] * framelist.channels
    sum0 = [2 ** 14] * framelist.channels
    sum1 = [2 ** 14] * framelist.channels

    #encode residuals
    for (i, pcm_frame) in enumerate(zip(*residuals)):
        for (c, residual) in enumerate(pcm_frame):
            #convert signed residual to unsigned
            if (residual > 0):
                unsigned = (residual * 2) - 1
            else:
                unsigned = (-residual) * 2

            if (unsigned < (2 ** k0[c])):
                writer.unary(0, 0)
                writer.write(k0[c], unsigned)
            else:
                shifted = (unsigned - (2 ** k0[c]))

                MSB = 1 + (shifted >> k1[c])
                LSB = shifted - ((MSB - 1) << k1[c])
                writer.unary(0, MSB)
                writer.write(k1[c], LSB)

                sum1[c] += (shifted - (sum1[c] >> 4))
                if (sum1[c] < (2 ** (k1[c] + 4))):
                    k1[c] = max(k1[c] - 1, 0)
                elif (sum1[c] > (2 ** (k1[c] + 5))):
                    k1[c] += 1

            #adjust sum0 and k0
            sum0[c] += unsigned - (sum0[c] >> 4)
            if (sum0[c] < (2 ** (k0[c] + 4))):
                k0[c] = max(k0[c] - 1, 0)
            elif (sum0[c] > (2 ** (k0[c] + 5))):
                k0[c] += 1

    #byte-align frame
    writer.byte_align()

    #write frame CRC32
    writer.pop_callback()
    writer.write(32, int(frame_crc))

    #return complete size of frame in bytes
    writer.pop_callback()
    return int(counter)


def correlate_channels(framelist):
    channels = len(framelist)
    pcm_frames = len(framelist[0])

    assert(channels > 1)

    correlated = []

    for c in xrange(channels):
        correlated_ch = []
        if (c == (channels - 1)):
            for i in xrange(pcm_frames):
                #round toward zero
                if (correlated[c - 1][i] >= 0):
                    correlated_ch.append(
                        framelist[c][i] - (correlated[c - 1][i] // 2))
                else:
                    correlated_ch.append(
                        framelist[c][i] - div_ceil(correlated[c - 1][i], 2))
        else:
            for i in xrange(pcm_frames):
                correlated_ch.append(framelist[c + 1][i] - framelist[c][i])
        correlated.append(correlated_ch)

    return correlated


def fixed_predictor(bits_per_sample, correlated):
    if (bits_per_sample == 8):
        shift = 4
    elif (bits_per_sample == 16):
        shift = 5
    elif (bits_per_sample == 24):
        shift = 5

    predicted = [correlated[0]]
    for i in xrange(1, len(correlated)):
        predicted.append(
            correlated[i] -
            (((correlated[i - 1] << shift) - correlated[i - 1]) >> shift))

    assert(len(predicted) == len(correlated))
    return predicted


def tta_filter(bps, predicted):
    if (bps == 8):
        shift = 10
    elif (bps == 16):
        shift = 9
    elif (bps == 24):
        shift = 10
    round_ = (1 << (shift - 1))

    residuals = []

    qm = [0] * 8
    dx = [0] * 8
    dl = [0] * 8

    for i in xrange(0, len(predicted)):
        if (i == 0):
            residuals.append(predicted[i] + (round_ >> shift))
        else:
            if (residuals[i - 1] < 0):
                qm = [m - x for (m, x) in zip(qm, dx)]
            elif (residuals[i - 1] > 0):
                qm = [m + x for (m, x) in zip(qm, dx)]

            sum_ = round_ + sum([l * m for (l, m) in zip(dl, qm)])

            #truncate sum to a 32-bit signed integer
            while (sum_ >= (2 ** 31)):
                sum_ -= (2 ** 32)
            while (sum_ < -(2 ** 31)):
                sum_ += (2 ** 32)

            residuals.append(predicted[i] - (sum_ >> shift))

        dx = [dx[1],
              dx[2],
              dx[3],
              dx[4],
              1 if (dl[4] >= 0) else -1,
              2 if (dl[5] >= 0) else -2,
              2 if (dl[6] >= 0) else -2,
              4 if (dl[7] >= 0) else -4]

        dl = [dl[1],
              dl[2],
              dl[3],
              dl[4],
              -dl[5] + (-dl[6] + (predicted[i] - dl[7])),
              -dl[6] + (predicted[i] - dl[7]),
              predicted[i] - dl[7],
              predicted[i]]

    assert(len(predicted) == len(residuals))
    return residuals


class Counter:
    def __init__(self):
        self.bytes = 0

    def update(self, b):
        self.bytes += 1

    def __int__(self):
        return self.bytes


class CRC32:
    TABLE = [0x00000000, 0x77073096, 0xee0e612c, 0x990951ba,
             0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
             0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
             0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
             0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de,
             0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
             0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,
             0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
             0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
             0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
             0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940,
             0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
             0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116,
             0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
             0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
             0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
             0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a,
             0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
             0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818,
             0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
             0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
             0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
             0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c,
             0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
             0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2,
             0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
             0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
             0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
             0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086,
             0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
             0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4,
             0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
             0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
             0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
             0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8,
             0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
             0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe,
             0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
             0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
             0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
             0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252,
             0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
             0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60,
             0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
             0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
             0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
             0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04,
             0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
             0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a,
             0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
             0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
             0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
             0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e,
             0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
             0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c,
             0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
             0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
             0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
             0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0,
             0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
             0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6,
             0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
             0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
             0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d]

    def __init__(self):
        self.crc = 0xffffffff

    def update(self, byte):
        self.crc = self.TABLE[(self.crc ^ byte) & 0xFF] ^ (self.crc >> 8)

    def __int__(self):
        return self.crc ^ 0xFFFFFFFF
