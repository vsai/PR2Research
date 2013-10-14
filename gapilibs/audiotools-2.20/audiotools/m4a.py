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


from audiotools import (AudioFile, InvalidFile, BIN, Image)
from .m4a_atoms import *

#######################
#M4A File
#######################


class InvalidM4A(InvalidFile):
    pass


def get_m4a_atom(reader, *atoms):
    """given a BitstreamReader and atom name strings
    returns a (size, substream) of the final atom data
    (not including its 64-bit size/name header)
    after traversing the parent atoms
    """

    from . import iter_last

    for (last, next_atom) in iter_last(iter(atoms)):
        try:
            (length, stream_atom) = reader.parse("32u 4b")
            while (stream_atom != next_atom):
                reader.skip_bytes(length - 8)
                (length, stream_atom) = reader.parse("32u 4b")
            if (last):
                return (length - 8, reader.substream(length - 8))
            else:
                reader = reader.substream(length - 8)
        except IOError:
            raise KeyError(next_atom)


def get_m4a_atom_offset(reader, *atoms):
    """given a BitstreamReader and atom name strings
    returns a (size, offset) of the final atom data
    (including its 64-bit size/name header)
    after traversing the parent atoms"""

    from . import iter_last

    offset = 0

    for (last, next_atom) in iter_last(iter(atoms)):
        try:
            (length, stream_atom) = reader.parse("32u 4b")
            offset += 8
            while (stream_atom != next_atom):
                reader.skip_bytes(length - 8)
                offset += (length - 8)
                (length, stream_atom) = reader.parse("32u 4b")
                offset += 8
            if (last):
                return (length, offset - 8)
            else:
                reader = reader.substream(length - 8)
        except IOError:
            raise KeyError(next_atom)


class M4ATaggedAudio:
    def __init__(self, filename):
        self.filename = filename

    def get_metadata(self):
        """returns a MetaData object, or None

        raises IOError if unable to read the file"""

        from .bitstream import BitstreamReader

        reader = BitstreamReader(file(self.filename, 'rb'), 0)
        try:
            try:
                (meta_size,
                 meta_reader) = get_m4a_atom(reader, "moov", "udta", "meta")
            except KeyError:
                return None

            return M4A_META_Atom.parse("meta", meta_size, meta_reader,
                                       {"hdlr": M4A_HDLR_Atom,
                                        "ilst": M4A_Tree_Atom,
                                        "free": M4A_FREE_Atom,
                                        "\xa9alb": M4A_ILST_Leaf_Atom,
                                        "\xa9ART": M4A_ILST_Leaf_Atom,
                                        'aART': M4A_ILST_Leaf_Atom,
                                        "\xa9cmt": M4A_ILST_Leaf_Atom,
                                        "covr": M4A_ILST_Leaf_Atom,
                                        "cpil": M4A_ILST_Leaf_Atom,
                                        "cprt": M4A_ILST_Leaf_Atom,
                                        "\xa9day": M4A_ILST_Leaf_Atom,
                                        "disk": M4A_ILST_Leaf_Atom,
                                        "gnre": M4A_ILST_Leaf_Atom,
                                        "----": M4A_ILST_Leaf_Atom,
                                        "pgap": M4A_ILST_Leaf_Atom,
                                        "rtng": M4A_ILST_Leaf_Atom,
                                        "tmpo": M4A_ILST_Leaf_Atom,
                                        "\xa9grp": M4A_ILST_Leaf_Atom,
                                        "\xa9nam": M4A_ILST_Leaf_Atom,
                                        "\xa9too": M4A_ILST_Leaf_Atom,
                                        "trkn": M4A_ILST_Leaf_Atom,
                                        "\xa9wrt": M4A_ILST_Leaf_Atom})
        finally:
            reader.close()

    def update_metadata(self, metadata, old_metadata=None):
        """takes this track's updated MetaData object
        as returned by get_metadata() and sets this track's metadata
        with any fields updated in that object

        old_metadata is the unmodifed metadata returned by get_metadata()

        raises IOError if unable to write the file
        """

        from .bitstream import BitstreamWriter
        from .bitstream import BitstreamReader
        import os.path

        if (metadata is None):
            return

        if (not isinstance(metadata, M4A_META_Atom)):
            from .text import ERR_FOREIGN_METADATA
            raise ValueError(ERR_FOREIGN_METADATA)

        if (old_metadata is None):
            #get_metadata() result may still be None, and that's okay
            old_metadata = self.get_metadata()

        #M4A streams often have *two* "free" atoms we can attempt to resize

        #first, attempt to resize the one inside the "meta" atom
        if ((old_metadata is not None) and
            metadata.has_child("free") and
            ((metadata.size() - metadata["free"].size()) <=
             old_metadata.size())):

            metadata.replace_child(
                M4A_FREE_Atom(old_metadata.size() -
                              (metadata.size() -
                               metadata["free"].size())))

            f = file(self.filename, 'r+b')
            (meta_size, meta_offset) = get_m4a_atom_offset(
                BitstreamReader(f, 0), "moov", "udta", "meta")
            f.seek(meta_offset + 8, 0)
            metadata.build(BitstreamWriter(f, 0))
            f.close()
            return
        else:
            #if there's insufficient room,
            #attempt to resize the outermost "free" also

            #this is only possible if the file is laid out correctly,
            #with "free" coming after "moov" but before "mdat"
            #FIXME

            #if neither fix is possible, the whole file must be rewritten
            #which also requires adjusting the "stco" atom offsets
            m4a_tree = M4A_Tree_Atom.parse(
                None,
                os.path.getsize(self.filename),
                BitstreamReader(file(self.filename, "rb"), 0),
                {"moov": M4A_Tree_Atom,
                 "trak": M4A_Tree_Atom,
                 "mdia": M4A_Tree_Atom,
                 "minf": M4A_Tree_Atom,
                 "stbl": M4A_Tree_Atom,
                 "stco": M4A_STCO_Atom,
                 "udta": M4A_Tree_Atom})

            #find initial mdat offset
            initial_mdat_offset = m4a_tree.child_offset("mdat")

            #adjust moov -> udta -> meta atom
            #(generating sub-atoms as necessary)
            if (not m4a_tree.has_child("moov")):
                return
            else:
                moov = m4a_tree["moov"]
            if (not moov.has_child("udta")):
                moov.append_child(M4A_Tree_Atom("udta", []))
            udta = moov["udta"]
            if (not udta.has_child("meta")):
                udta.append_child(metadata)
            else:
                udta.replace_child(metadata)

            #find new mdat offset
            new_mdat_offset = m4a_tree.child_offset("mdat")

            #adjust moov -> trak -> mdia -> minf -> stbl -> stco offsets
            #based on the difference between the new mdat position and the old
            try:
                delta_offset = new_mdat_offset - initial_mdat_offset
                stco = m4a_tree["moov"]["trak"]["mdia"]["minf"]["stbl"]["stco"]
                stco.offsets = [offset + delta_offset for offset in
                                stco.offsets]
            except KeyError:
                #if there is no stco atom, don't worry about it
                pass

            #then write entire tree back to disk
            writer = BitstreamWriter(file(self.filename, "wb"), 0)
            m4a_tree.build(writer)

    def set_metadata(self, metadata):
        """takes a MetaData object and sets this track's metadata

        this metadata includes track name, album name, and so on
        raises IOError if unable to write the file"""

        if (metadata is None):
            return

        old_metadata = self.get_metadata()
        metadata = M4A_META_Atom.converted(metadata)

        #replace file-specific atoms in new metadata
        #with ones from old metadata (if any)
        #which can happen if we're shifting metadata
        #from one M4A file to another
        file_specific_atoms = frozenset(['\xa9too', '----', 'pgap', 'tmpo'])

        if (metadata.has_ilst_atom()):
            metadata.ilst_atom().leaf_atoms = filter(
                lambda atom: atom.name not in file_specific_atoms,
                metadata.ilst_atom())

            if (old_metadata.has_ilst_atom()):
                metadata.ilst_atom().leaf_atoms.extend(
                    filter(lambda atom: atom.name in file_specific_atoms,
                           old_metadata.ilst_atom()))

        self.update_metadata(metadata, old_metadata)

    def delete_metadata(self):
        """deletes the track's MetaData

        this removes or unsets tags as necessary in order to remove all data
        raises IOError if unable to write the file"""

        from . import MetaData

        self.set_metadata(MetaData())


class M4AAudio_faac(M4ATaggedAudio, AudioFile):
    """an M4A audio file using faac/faad binaries for I/O"""

    SUFFIX = "m4a"
    NAME = SUFFIX
    DESCRIPTION = u"Advanced Audio Coding"
    DEFAULT_COMPRESSION = "100"
    COMPRESSION_MODES = tuple(["10"] + map(str, range(50, 500, 25)) + ["500"])
    BINARIES = ("faac", "faad")
    BINARY_URLS = {"faac": "http://www.audiocoding.com/",
                   "faad": "http://www.audiocoding.com/"}

    def __init__(self, filename):
        """filename is a plain string"""

        from .bitstream import BitstreamReader

        self.filename = filename

        #first, fetch the mdia atom
        #which is the parent of both the mp4a and mdhd atoms
        try:
            mdia = get_m4a_atom(BitstreamReader(file(filename, 'rb'), 0),
                                "moov", "trak", "mdia")[1]
        except IOError:
            from .text import ERR_M4A_IOERROR
            raise InvalidM4A(ERR_M4A_IOERROR)
        except KeyError:
            from .text import ERR_M4A_MISSING_MDIA
            raise InvalidM4A(ERR_M4A_MISSING_MDIA)
        mdia.mark()
        try:
            try:
                stsd = get_m4a_atom(mdia, "minf", "stbl", "stsd")[1]
            except KeyError:
                from .text import ERR_M4A_MISSING_STSD
                raise InvalidM4A(ERR_M4A_MISSING_STSD)

            #then, fetch the mp4a atom for bps, channels and sample rate
            try:
                (stsd_version, descriptions) = stsd.parse("8u 24p 32u")
                (mp4a,
                 self.__channels__,
                 self.__bits_per_sample__) = stsd.parse(
                     "32p 4b 48p 16p 16p 16p 4P 16u 16u 16p 16p 32p")
            except IOError:
                from .text import ERR_M4A_INVALID_MP4A
                raise InvalidM4A(ERR_M4A_INVALID_MP4A)

            #finally, fetch the mdhd atom for total track length
            mdia.rewind()
            try:
                mdhd = get_m4a_atom(mdia, "mdhd")[1]
            except KeyError:
                from .text import ERR_M4A_MISSING_MDHD
                raise InvalidM4A(ERR_M4A_MISSING_MDHD)
            try:
                (version, ) = mdhd.parse("8u 24p")
                if (version == 0):
                    (self.__sample_rate__,
                     self.__length__,) = mdhd.parse("32p 32p 32u 32u 2P 16p")
                elif (version == 1):
                    (self.__sample_rate__,
                     self.__length__,) = mdhd.parse("64p 64p 32u 64U 2P 16p")
                else:
                    from .text import ERR_M4A_UNSUPPORTED_MDHD
                    raise InvalidM4A(ERR_M4A_UNSUPPORTED_MDHD)
            except IOError:
                from .text import ERR_M4A_INVALID_MDHD
                raise InvalidM4A(ERR_M4A_INVALID_MDHD)
        finally:
            mdia.unmark()

    def channel_mask(self):
        """returns a ChannelMask object of this track's channel layout"""

        from . import ChannelMask

        #M4A seems to use the same channel assignment
        #as old-style RIFF WAVE/FLAC
        if (self.channels() == 1):
            return ChannelMask.from_fields(
                front_center=True)
        elif (self.channels() == 2):
            return ChannelMask.from_fields(
                front_left=True, front_right=True)
        elif (self.channels() == 3):
            return ChannelMask.from_fields(
                front_left=True, front_right=True, front_center=True)
        elif (self.channels() == 4):
            return ChannelMask.from_fields(
                front_left=True, front_right=True,
                back_left=True, back_right=True)
        elif (self.channels() == 5):
            return ChannelMask.from_fields(
                front_left=True, front_right=True, front_center=True,
                back_left=True, back_right=True)
        elif (self.channels() == 6):
            return ChannelMask.from_fields(
                front_left=True, front_right=True, front_center=True,
                back_left=True, back_right=True,
                low_frequency=True)
        else:
            return ChannelMask(0)

    def lossless(self):
        """returns False"""

        return False

    def channels(self):
        """returns an integer number of channels this track contains"""

        return self.__channels__

    def bits_per_sample(self):
        """returns an integer number of bits-per-sample this track contains"""

        return self.__bits_per_sample__

    def sample_rate(self):
        """returns the rate of the track's audio as an integer number of Hz"""

        return self.__sample_rate__

    def cd_frames(self):
        """returns the total length of the track in CD frames

        each CD frame is 1/75th of a second"""

        return (self.__length__ - 1024) / self.__sample_rate__ * 75

    def total_frames(self):
        """returns the total PCM frames of the track as an integer"""

        return self.__length__ - 1024

    def to_pcm(self):
        """returns a PCMReader object containing the track's PCM data"""

        from . import PCMReader
        import subprocess
        import os

        devnull = file(os.devnull, "ab")

        sub = subprocess.Popen([BIN['faad'], "-f", str(2), "-w",
                                self.filename],
                               stdout=subprocess.PIPE,
                               stderr=devnull)
        return PCMReader(sub.stdout,
                         sample_rate=self.sample_rate(),
                         channels=self.channels(),
                         channel_mask=int(self.channel_mask()),
                         bits_per_sample=self.bits_per_sample(),
                         process=sub)

    @classmethod
    def from_pcm(cls, filename, pcmreader,
                 compression=None, total_pcm_frames=None):
        """encodes a new file from PCM data

        takes a filename string, PCMReader object,
        optional compression level string and optional
        total_pcm_frames integer
        encodes a new audio file from pcmreader's data
        at the given filename with the specified compression level
        and returns a new M4AAudio object"""

        from . import PCMConverter
        from . import transfer_data
        from . import transfer_framelist_data
        from . import ignore_sigint
        from . import EncodingError
        from . import DecodingError
        from . import ChannelMask
        from . import __default_quality__
        import subprocess
        import os

        if ((compression is None) or (compression not in
                                      cls.COMPRESSION_MODES)):
            compression = __default_quality__(cls.NAME)

        if (pcmreader.channels > 2):
            pcmreader = PCMConverter(pcmreader,
                                     sample_rate=pcmreader.sample_rate,
                                     channels=2,
                                     channel_mask=ChannelMask.from_channels(2),
                                     bits_per_sample=pcmreader.bits_per_sample)

        #faac requires files to end with .m4a for some reason
        if (not filename.endswith(".m4a")):
            import tempfile
            actual_filename = filename
            tempfile = tempfile.NamedTemporaryFile(suffix=".m4a")
            filename = tempfile.name
        else:
            actual_filename = tempfile = None

        devnull = file(os.devnull, "ab")

        sub = subprocess.Popen([BIN['faac'],
                                "-q", compression,
                                "-P",
                                "-R", str(pcmreader.sample_rate),
                                "-B", str(pcmreader.bits_per_sample),
                                "-C", str(pcmreader.channels),
                                "-X",
                                "-o", filename,
                                "-"],
                               stdin=subprocess.PIPE,
                               stderr=devnull,
                               stdout=devnull,
                               preexec_fn=ignore_sigint)
        #Note: faac handles SIGINT on its own,
        #so trying to ignore it doesn't work like on most other encoders.

        try:
            transfer_framelist_data(pcmreader, sub.stdin.write)
        except (ValueError, IOError), err:
            sub.stdin.close()
            sub.wait()
            cls.__unlink__(filename)
            raise EncodingError(str(err))
        except Exception, err:
            sub.stdin.close()
            sub.wait()
            cls.__unlink__(filename)
            raise err

        try:
            pcmreader.close()
        except DecodingError, err:
            raise EncodingError(err.error_message)
        sub.stdin.close()

        if (sub.wait() == 0):
            if (tempfile is not None):
                filename = actual_filename
                f = file(filename, 'wb')
                tempfile.seek(0, 0)
                transfer_data(tempfile.read, f.write)
                f.close()
                tempfile.close()

            return M4AAudio(filename)
        else:
            if (tempfile is not None):
                tempfile.close()
            raise EncodingError(u"unable to write file with faac")

    @classmethod
    def supports_replay_gain(cls):
        """returns True if this class supports ReplayGain"""

        return False

    @classmethod
    def can_add_replay_gain(cls, audiofiles):
        """returns False"""

        return False

    @classmethod
    def lossless_replay_gain(cls):
        """returns False"""

        return False

    @classmethod
    def add_replay_gain(cls, filenames, progress=None):
        """adds ReplayGain values to a list of filename strings

        all the filenames must be of this AudioFile type
        raises ValueError if some problem occurs during ReplayGain application
        """

        import subprocess
        import os
        from . import open_files

        track_names = [track.filename for track in
                       open_files(filenames) if
                       isinstance(track, cls)]

        if (progress is not None):
            progress(0, 1)

        #helpfully, aacgain is flag-for-flag compatible with mp3gain
        if ((len(track_names) > 0) and (BIN.can_execute(BIN['aacgain']))):
            devnull = file(os.devnull, 'ab')
            sub = subprocess.Popen([BIN['aacgain'], '-k', '-q', '-r'] +
                                   track_names,
                                   stdout=devnull,
                                   stderr=devnull)
            sub.wait()

            devnull.close()

        if (progress is not None):
            progress(1, 1)


class M4AAudio_nero(M4AAudio_faac):
    """an M4A audio file using neroAacEnc/neroAacDec binaries for I/O"""

    from .text import (COMP_NERO_LOW, COMP_NERO_HIGH)

    DEFAULT_COMPRESSION = "0.5"
    COMPRESSION_MODES = ("0.4", "0.5",
                         "0.6", "0.7", "0.8", "0.9", "1.0")
    COMPRESSION_DESCRIPTIONS = {"0.4": COMP_NERO_LOW,
                                "1.0": COMP_NERO_HIGH}
    BINARIES = ("neroAacDec", "neroAacEnc")
    BINARY_URLS = {"neroAacDec": "http://www.nero.com/enu/" +
                   "downloads-nerodigital-nero-aac-codec.php",
                   "neroAacEnc": "http://www.nero.com/enu/" +
                   "downloads-nerodigital-nero-aac-codec.php"}

    @classmethod
    def from_pcm(cls, filename, pcmreader,
                 compression=None, total_pcm_frames=None):
        """encodes a new file from PCM data

        takes a filename string, PCMReader object,
        optional compression level string and
        optional total_pcm_frames integer
        encodes a new audio file from pcmreader's data
        at the given filename with the specified compression level
        and returns a new M4AAudio object"""

        import tempfile
        import os
        import os.path
        from . import PCMConverter
        from . import WaveAudio
        from . import __default_quality__

        if ((compression is None) or (compression not in
                                      cls.COMPRESSION_MODES)):
            compression = __default_quality__(cls.NAME)

        tempwavefile = tempfile.NamedTemporaryFile(suffix=".wav")
        try:
            if (pcmreader.sample_rate > 96000):
                tempwave = WaveAudio.from_pcm(
                    tempwavefile.name,
                    PCMConverter(pcmreader,
                                 sample_rate=96000,
                                 channels=pcmreader.channels,
                                 channel_mask=pcmreader.channel_mask,
                                 bits_per_sample=pcmreader.bits_per_sample))
            else:
                tempwave = WaveAudio.from_pcm(
                    tempwavefile.name,
                    pcmreader)

            cls.__from_wave__(filename, tempwave.filename, compression)
            return cls(filename)
        finally:
            if (os.path.isfile(tempwavefile.name)):
                tempwavefile.close()
            else:
                tempwavefile.close_called = True

    def to_pcm(self):
        import tempfile
        from . import EncodingError
        from . import PCMReaderError
        from .wav import TempWaveReader

        f = tempfile.NamedTemporaryFile(suffix=".wav")
        try:
            self.__to_wave__(f.name)
            return TempWaveReader(f)
        except EncodingError, err:
            return PCMReaderError(error_message=err.error_message,
                                  sample_rate=self.sample_rate(),
                                  channels=self.channels(),
                                  channel_mask=int(self.channel_mask()),
                                  bits_per_sample=self.bits_per_sample())

    def __to_wave__(self, wave_file, progress=None):
        """writes the contents of this file to the given .wav filename string

        raises EncodingError if some error occurs during decoding"""

        import subprocess
        import os
        from . import EncodingError

        devnull = file(os.devnull, "w")
        try:
            sub = subprocess.Popen([BIN["neroAacDec"],
                                    "-if", self.filename,
                                    "-of", wave_file],
                                   stdout=devnull,
                                   stderr=devnull)
            if (sub.wait() != 0):
                raise EncodingError(u"unable to write file with neroAacDec")
        finally:
            devnull.close()

    @classmethod
    def __from_wave__(cls, filename, wave_filename, compression):
        import subprocess
        import os
        from . import EncodingError

        devnull = file(os.devnull, "w")
        try:
            sub = subprocess.Popen([BIN["neroAacEnc"],
                                    "-q", compression,
                                    "-if", wave_filename,
                                    "-of", filename],
                                   stdout=devnull,
                                   stderr=devnull)

            if (sub.wait() != 0):
                raise EncodingError(u"neroAacEnc unable to write file")
            else:
                return cls(filename)
        finally:
            devnull.close()

if (BIN.can_execute(BIN["neroAacEnc"]) and BIN.can_execute(BIN["neroAacDec"])):
    M4AAudio = M4AAudio_nero
else:
    M4AAudio = M4AAudio_faac


class M4ACovr(Image):
    """a subclass of Image to store M4A 'covr' atoms"""

    def __init__(self, image_data):
        self.image_data = image_data

        img = Image.new(image_data, u'', 0)

        Image.__init__(self,
                       data=image_data,
                       mime_type=img.mime_type,
                       width=img.width,
                       height=img.height,
                       color_depth=img.color_depth,
                       color_count=img.color_count,
                       description=img.description,
                       type=img.type)

    @classmethod
    def converted(cls, image):
        """given an Image object, returns an M4ACovr object"""

        return M4ACovr(image.data)


class __counter__:
    def __init__(self):
        self.val = 0

    def incr(self):
        self.val += 1

    def __int__(self):
        return self.val


class InvalidALAC(InvalidFile):
    pass


class ALACAudio(M4ATaggedAudio, AudioFile):
    """an Apple Lossless audio file"""

    SUFFIX = "m4a"
    NAME = "alac"
    DESCRIPTION = u"Apple Lossless"
    DEFAULT_COMPRESSION = ""
    COMPRESSION_MODES = ("",)
    BINARIES = tuple()

    BLOCK_SIZE = 4096
    INITIAL_HISTORY = 10
    HISTORY_MULTIPLIER = 40
    MAXIMUM_K = 14

    def __init__(self, filename):
        """filename is a plain string"""

        from .bitstream import BitstreamReader

        self.filename = filename

        #first, fetch the mdia atom
        #which is the parent of both the alac and mdhd atoms
        try:
            mdia = get_m4a_atom(BitstreamReader(file(filename, 'rb'), 0),
                                "moov", "trak", "mdia")[1]
        except IOError:
            from .text import ERR_ALAC_IOERROR
            raise InvalidALAC(ERR_ALAC_IOERROR)
        except KeyError:
            from .text import ERR_M4A_MISSING_MDIA
            raise InvalidALAC(ERR_M4A_MISSING_MDIA)
        mdia.mark()
        try:
            try:
                stsd = get_m4a_atom(mdia, "minf", "stbl", "stsd")[1]
            except KeyError:
                from .text import ERR_M4A_MISSING_STSD
                raise InvalidALAC(ERR_M4A_MISSING_STSD)

            #then, fetch the alac atom for bps, channels and sample rate
            try:
                #though some of these fields are parsed redundantly
                #in .to_pcm(), we still need to parse them here
                #to fetch values for .bits_per_sample(), etc.
                (stsd_version, descriptions) = stsd.parse("8u 24p 32u")
                (alac1,
                 alac2,
                 self.__max_samples_per_frame__,
                 self.__bits_per_sample__,
                 self.__history_multiplier__,
                 self.__initial_history__,
                 self.__maximum_k__,
                 self.__channels__,
                 self.__sample_rate__) = stsd.parse(
                     #ignore much of the stuff in the "high" ALAC atom
                     "32p 4b 6P 16p 16p 16p 4P 16p 16p 16p 16p 4P" +
                     #and use the attributes in the "low" ALAC atom instead
                     "32p 4b 4P 32u 8p 8u 8u 8u 8u 8u 16p 32p 32p 32u")
            except IOError:
                from .text import ERR_ALAC_INVALID_ALAC
                raise InvalidALAC(ERR_ALAC_INVALID_ALAC)

            if ((alac1 != 'alac') or (alac2 != 'alac')):
                from .text import ERR_ALAC_INVALID_ALAC
                mdia.unmark()
                raise InvalidALAC(ERR_ALAC_INVALID_ALAC)

            #finally, fetch the mdhd atom for total track length
            mdia.rewind()
            try:
                mdhd = get_m4a_atom(mdia, "mdhd")[1]
            except KeyError:
                from .text import ERR_M4A_MISSING_MDHD
                raise InvalidALAC(ERR_M4A_MISSING_MDHD)
            try:
                (version, ) = mdhd.parse("8u 24p")
                if (version == 0):
                    (self.__length__,) = mdhd.parse("32p 32p 32p 32u 2P 16p")
                elif (version == 1):
                    (self.__length__,) = mdhd.parse("64p 64p 32p 64U 2P 16p")
                else:
                    from .text import ERR_M4A_UNSUPPORTED_MDHD
                    raise InvalidALAC(ERR_M4A_UNSUPPORTED_MDHD)
            except IOError:
                from .text import ERR_M4A_INVALID_MDHD
                raise InvalidALAC(ERR_M4A_INVALID_MDHD)
        finally:
            mdia.unmark()

    def channels(self):
        """returns an integer number of channels this track contains"""

        return self.__channels__

    def bits_per_sample(self):
        """returns an integer number of bits-per-sample this track contains"""

        return self.__bits_per_sample__

    def sample_rate(self):
        """returns the rate of the track's audio as an integer number of Hz"""

        return self.__sample_rate__

    def total_frames(self):
        """returns the total PCM frames of the track as an integer"""

        return self.__length__

    def channel_mask(self):
        """returns a ChannelMask object of this track's channel layout"""

        from . import ChannelMask

        return {
            1: ChannelMask.from_fields(
                front_center=True),
            2: ChannelMask.from_fields(
                front_left=True,
                front_right=True),
            3: ChannelMask.from_fields(
                front_center=True,
                front_left=True,
                front_right=True),
            4: ChannelMask.from_fields(
                front_center=True,
                front_left=True,
                front_right=True,
                back_center=True),
            5: ChannelMask.from_fields(
                front_center=True,
                front_left=True,
                front_right=True,
                back_left=True,
                back_right=True),
            6: ChannelMask.from_fields(
                front_center=True,
                front_left=True,
                front_right=True,
                back_left=True,
                back_right=True,
                low_frequency=True),
            7: ChannelMask.from_fields(
                front_center=True,
                front_left=True,
                front_right=True,
                back_left=True,
                back_right=True,
                back_center=True,
                low_frequency=True),
            8: ChannelMask.from_fields(
                front_center=True,
                front_left_of_center=True,
                front_right_of_center=True,
                front_left=True,
                front_right=True,
                back_left=True,
                back_right=True,
                low_frequency=True)}.get(self.channels(), ChannelMask(0))

    def cd_frames(self):
        """returns the total length of the track in CD frames

        each CD frame is 1/75th of a second"""

        try:
            return (self.total_frames() * 75) / self.sample_rate()
        except ZeroDivisionError:
            return 0

    def lossless(self):
        """returns True"""

        return True

    def to_pcm(self):
        """returns a PCMReader object containing the track's PCM data"""

        from .decoders import ALACDecoder
        from . import PCMReaderError

        try:
            return ALACDecoder(self.filename)
        except (IOError, ValueError), msg:
            return PCMReaderError(error_message=str(msg),
                                  sample_rate=self.sample_rate(),
                                  channels=self.channels(),
                                  channel_mask=int(self.channel_mask()),
                                  bits_per_sample=self.bits_per_sample())

    @classmethod
    def from_pcm(cls, filename, pcmreader,
                 compression=None, total_pcm_frames=None,
                 block_size=4096, encoding_function=None):
        """encodes a new file from PCM data

        takes a filename string, PCMReader object,
        optional compression level string and
        optional total_pcm_frames integer
        encodes a new audio file from pcmreader's data
        at the given filename with the specified compression level
        and returns a new ALACAudio object"""

        if (pcmreader.bits_per_sample not in (16, 24)):
            from . import UnsupportedBitsPerSample

            raise UnsupportedBitsPerSample(filename, pcmreader.bits_per_sample)

        if (int(pcmreader.channel_mask) not in
            (0x0001,    # 1ch - mono
             0x0004,    # 1ch - mono
             0x0003,    # 2ch - left, right
             0x0007,    # 3ch - center, left, right
             0x0107,    # 4ch - center, left, right, back center
             0x0037,    # 5ch - center, left, right, back left, back right
             0x003F,    # 6ch - C, L, R, back left, back right, LFE
             0x013F,    # 7ch - C, L, R, bL, bR, back center, LFE
             0x00FF,    # 8ch - C, cL, cR, L, R, bL, bR, LFE
             0x0000)):  # undefined
            from . import UnsupportedChannelMask

            raise UnsupportedChannelMask(filename,
                                         int(pcmreader.channel_mask))

        from .encoders import encode_alac
        from .bitstream import BitstreamWriter
        from . import transfer_data
        from . import EncodingError
        from . import BufferedPCMReader
        from . import at_a_time
        import time
        import tempfile

        ftyp = cls.__ftyp_atom__()
        free = cls.__free_atom__(0x1000)
        create_date = long(time.time()) + 2082844800

        if (total_pcm_frames is not None):
            total_alac_frames = ((total_pcm_frames // block_size) +
                                 (1 if (total_pcm_frames % block_size) else 0))

            #build a set of placeholder atoms
            #to stick at the start of the file
            moov = cls.__moov_atom__(pcmreader,
                                     create_date,
                                     0,  # placeholder
                                     0,  # placeholder
                                     block_size,
                                     total_pcm_frames,
                                     [0] * total_alac_frames)

            try:
                f = file(filename, 'wb')
            except IOError, err:
                raise EncodingError(str(err))
            try:
                m4a_writer = BitstreamWriter(f, 0)
                m4a_writer.build("32u 4b", (ftyp.size() + 8, ftyp.name))
                ftyp.build(m4a_writer)
                m4a_writer.build("32u 4b", (moov.size() + 8, moov.name))
                moov.build(m4a_writer)
                m4a_writer.build("32u 4b", (free.size() + 8, free.name))
                free.build(m4a_writer)
            except (IOError), err:
                f.close()
                self.__unlink__(filename)
                raise EncodingError(str(err))

            #encode the mdat atom based on encoding parameters
            try:
                (frame_byte_sizes, actual_pcm_frames) = \
                    (encode_alac if encoding_function is None else
                     encoding_function)(
                         file=f,
                         pcmreader=BufferedPCMReader(pcmreader),
                         block_size=block_size,
                         initial_history=cls.INITIAL_HISTORY,
                         history_multiplier=cls.HISTORY_MULTIPLIER,
                         maximum_k=cls.MAXIMUM_K)
            except (IOError, ValueError), err:
                f.close()
                self.__unlink__(filename)
                raise EncodingError(str(err))

            if (actual_pcm_frames != total_pcm_frames):
                from .text import ERR_TOTAL_PCM_FRAMES_MISMATCH
                raise EncodingError(ERR_TOTAL_PCM_FRAMES_MISMATCH)
            assert(sum(frame_byte_sizes) > 0)

            mdat_size = 8 + sum(frame_byte_sizes)
            pre_mdat_size = (8 + ftyp.size() +
                             8 + moov.size() +
                             8 + free.size())

            #go back and re-populate placeholder atoms
            #with actual values
            moov = cls.__moov_atom__(pcmreader,
                                     create_date,
                                     pre_mdat_size,
                                     mdat_size,
                                     block_size,
                                     total_pcm_frames,
                                     frame_byte_sizes)

            f.seek(0, 0)
            m4a_writer.build("32u 4b", (ftyp.size() + 8, ftyp.name))
            ftyp.build(m4a_writer)
            m4a_writer.build("32u 4b", (moov.size() + 8, moov.name))
            moov.build(m4a_writer)
            f.close()

            return cls(filename)
        else:
            mdat_file = tempfile.TemporaryFile()

            #perform encode_alac() on pcmreader to our output file
            #which returns a tuple of output values
            #which are various fields for the "alac" atom
            try:
                (frame_byte_sizes, total_pcm_frames) = \
                    (encode_alac if encoding_function is None else
                     encoding_function)(
                         file=mdat_file,
                         pcmreader=BufferedPCMReader(pcmreader),
                         block_size=block_size,
                         initial_history=cls.INITIAL_HISTORY,
                         history_multiplier=cls.HISTORY_MULTIPLIER,
                         maximum_k=cls.MAXIMUM_K)
            except (IOError, ValueError), err:
                raise EncodingError(str(err))

            mdat_size = 8 + sum(frame_byte_sizes)

            #use the fields from encode_alac() to populate our ALAC atoms
            moov = cls.__moov_atom__(pcmreader,
                                     create_date,
                                     0,  # placeholder
                                     mdat_size,
                                     block_size,
                                     total_pcm_frames,
                                     frame_byte_sizes)

            #add the size of ftyp + moov + free to our absolute file offsets
            pre_mdat_size = (8 + ftyp.size() +
                             8 + moov.size() +
                             8 + free.size())

            #then regenerate the moov atom with actual data
            moov = cls.__moov_atom__(pcmreader,
                                     create_date,
                                     pre_mdat_size,
                                     mdat_size,
                                     block_size,
                                     total_pcm_frames,
                                     frame_byte_sizes)

            #build our complete output file
            try:
                f = file(filename, 'wb')
                m4a_writer = BitstreamWriter(f, 0)
                m4a_writer.build("32u 4b", (ftyp.size() + 8, ftyp.name))
                ftyp.build(m4a_writer)
                m4a_writer.build("32u 4b", (moov.size() + 8, moov.name))
                moov.build(m4a_writer)
                m4a_writer.build("32u 4b", (free.size() + 8, free.name))
                free.build(m4a_writer)
                mdat_file.seek(0, 0)
                transfer_data(mdat_file.read, f.write)
                mdat_file.close()
            except (IOError), err:
                mdat_file.close()
                raise EncodingError(str(err))

            return cls(filename)

    @classmethod
    def __ftyp_atom__(cls):
        return M4A_FTYP_Atom(major_brand='M4A ',
                             major_brand_version=0,
                             compatible_brands=['M4A ',
                                                'mp42',
                                                'isom',
                                                chr(0) * 4])

    @classmethod
    def __moov_atom__(cls, pcmreader,
                      create_date,
                      mdat_offset,
                      mdat_size,
                      block_size,
                      total_pcm_frames,
                      frame_byte_sizes):
        """pcmreader is a PCMReader object
        create_date is the file's creation time, in the Apple epoch, as an int
        mdat_offset is the number of bytes before the start of the mdat atom
        mdat_size is the complete size of the mdat atom, in bytes
        block_size is the requested size of each ALAC frame, in PCM frames
        total_pcm_frames is the total size of the file, in PCM frames
        """

        return M4A_Tree_Atom(
            "moov",
            [cls.__mvhd_atom__(pcmreader, create_date, total_pcm_frames),
             M4A_Tree_Atom(
                 "trak",
                 [cls.__tkhd_atom__(create_date, total_pcm_frames),
                  M4A_Tree_Atom(
                      "mdia",
                      [cls.__mdhd_atom__(pcmreader,
                                         create_date,
                                         total_pcm_frames),
                       cls.__hdlr_atom__(),
                       M4A_Tree_Atom("minf",
                                     [cls.__smhd_atom__(),
                                      M4A_Tree_Atom(
                                          "dinf",
                                          [cls.__dref_atom__()]),
                                      M4A_Tree_Atom(
                                          "stbl",
                                          [cls.__stsd_atom__(
                                              pcmreader,
                                              mdat_size,
                                              block_size,
                                              total_pcm_frames,
                                              frame_byte_sizes),
                                           cls.__stts_atom__(
                                               total_pcm_frames,
                                               block_size),
                                           cls.__stsc_atom__(
                                               total_pcm_frames,
                                               block_size),
                                           cls.__stsz_atom__(
                                               frame_byte_sizes),
                                           cls.__stco_atom__(
                                               mdat_offset,
                                               frame_byte_sizes)])])])]),
             M4A_Tree_Atom("udta", [cls.__meta_atom__()])])

    @classmethod
    def __mvhd_atom__(cls, pcmreader, create_date, total_pcm_frames):
        return M4A_MVHD_Atom(version=0,
                             flags=0,
                             created_utc_date=create_date,
                             modified_utc_date=create_date,
                             time_scale=pcmreader.sample_rate,
                             duration=total_pcm_frames,
                             playback_speed=0x10000,
                             user_volume=0x100,
                             geometry_matrices=[0x10000,
                                                0,
                                                0,
                                                0,
                                                0x10000,
                                                0,
                                                0,
                                                0,
                                                0x40000000],
                             qt_preview=0,
                             qt_still_poster=0,
                             qt_selection_time=0,
                             qt_current_time=0,
                             next_track_id=2)

    @classmethod
    def __tkhd_atom__(cls, create_date, total_pcm_frames):
        return M4A_TKHD_Atom(version=0,
                             track_in_poster=0,
                             track_in_preview=1,
                             track_in_movie=1,
                             track_enabled=1,
                             created_utc_date=create_date,
                             modified_utc_date=create_date,
                             track_id=1,
                             duration=total_pcm_frames,
                             video_layer=0,
                             qt_alternate=0,
                             volume=0x100,
                             geometry_matrices=[0x10000,
                                                0,
                                                0,
                                                0,
                                                0x10000,
                                                0,
                                                0,
                                                0,
                                                0x40000000],
                             video_width=0,
                             video_height=0)

    @classmethod
    def __mdhd_atom__(cls, pcmreader, create_date, total_pcm_frames):
        return M4A_MDHD_Atom(version=0,
                             flags=0,
                             created_utc_date=create_date,
                             modified_utc_date=create_date,
                             sample_rate=pcmreader.sample_rate,
                             track_length=total_pcm_frames,
                             language=[ord(c) - 0x60 for c in "und"],
                             quality=0)

    @classmethod
    def __hdlr_atom__(cls):
        return M4A_HDLR_Atom(version=0,
                             flags=0,
                             qt_type=chr(0) * 4,
                             qt_subtype='soun',
                             qt_manufacturer=chr(0) * 4,
                             qt_reserved_flags=0,
                             qt_reserved_flags_mask=0,
                             component_name="",
                             padding_size=1)

    @classmethod
    def __smhd_atom__(cls):
        return M4A_SMHD_Atom(version=0,
                             flags=0,
                             audio_balance=0)

    @classmethod
    def __dref_atom__(cls):
        return M4A_DREF_Atom(version=0,
                             flags=0,
                             references=[M4A_Leaf_Atom("url ",
                                                       "\x00\x00\x00\x01")])

    @classmethod
    def __stsd_atom__(cls, pcmreader,
                      mdat_size,
                      block_size,
                      total_pcm_frames,
                      frame_byte_sizes):
        return M4A_STSD_Atom(
            version=0,
            flags=0,
            descriptions=[
                M4A_ALAC_Atom(
                    reference_index=1,
                    qt_version=0,
                    qt_revision_level=0,
                    qt_vendor=chr(0) * 4,
                    channels=pcmreader.channels,
                    bits_per_sample=pcmreader.bits_per_sample,
                    qt_compression_id=0,
                    audio_packet_size=0,
                    sample_rate=0xAC440000,  # regardless of actual sample rate
                    sub_alac=M4A_SUB_ALAC_Atom(
                        max_samples_per_frame=block_size,
                        bits_per_sample=pcmreader.bits_per_sample,
                        history_multiplier=cls.HISTORY_MULTIPLIER,
                        initial_history=cls.INITIAL_HISTORY,
                        maximum_k=cls.MAXIMUM_K,
                        channels=pcmreader.channels,
                        unknown=0x00FF,
                        max_coded_frame_size=max(frame_byte_sizes),
                        bitrate=((mdat_size * 8 * pcmreader.sample_rate) //
                                 total_pcm_frames),
                        sample_rate=pcmreader.sample_rate))])

    @classmethod
    def __stts_atom__(cls, total_pcm_frames, block_size):
        #note that the first entry may have 0 items
        #and the second may have a duration of 0 PCM frames
        times = [(total_pcm_frames // block_size, block_size),
                 (1, total_pcm_frames % block_size)]

        return M4A_STTS_Atom(
            version=0,
            flags=0,
            #filter invalid times entries
            times=[t for t in times if ((t[0] > 0) and t[1] > 0)])

    @classmethod
    def __stsc_atom__(cls, total_pcm_frames, block_size):
        alac_frames = ((total_pcm_frames // block_size) +
                       (1 if (total_pcm_frames % block_size) else 0))
        alac_frames_per_chunk = 5

        if (alac_frames < alac_frames_per_chunk):
            blocks = [(1, alac_frames, 1)]
        else:
            blocks = [(1, alac_frames_per_chunk, 1)]
            if (alac_frames % alac_frames_per_chunk):
                blocks.append((1 + (alac_frames // alac_frames_per_chunk),
                               alac_frames % alac_frames_per_chunk,
                               1))

        return M4A_STSC_Atom(
            version=0,
            flags=0,
            blocks=blocks)

    @classmethod
    def __stsz_atom__(cls, frame_byte_sizes):
        return M4A_STSZ_Atom(
            version=0,
            flags=0,
            byte_size=0,
            block_sizes=frame_byte_sizes)

    @classmethod
    def __stco_atom__(cls, mdat_offset, frame_byte_sizes):
        alac_frames_per_chunk = 5
        frame_byte_sizes = frame_byte_sizes[:]
        chunk_offsets = [mdat_offset + 8]
        while (len(frame_byte_sizes) > 0):
            chunk_size = sum(frame_byte_sizes[0:alac_frames_per_chunk])
            chunk_offsets.append(chunk_offsets[-1] + chunk_size)
            frame_byte_sizes = frame_byte_sizes[alac_frames_per_chunk:]

        return M4A_STCO_Atom(
            version=0,
            flags=0,
            offsets=chunk_offsets[0:-1])

    @classmethod
    def __meta_atom__(cls):
        from . import VERSION

        return M4A_META_Atom(
            version=0,
            flags=0,
            leaf_atoms=[
                M4A_HDLR_Atom(version=0,
                              flags=0,
                              qt_type=chr(0) * 4,
                              qt_subtype='mdir',
                              qt_manufacturer='appl',
                              qt_reserved_flags=0,
                              qt_reserved_flags_mask=0,
                              component_name="",
                              padding_size=1),
                M4A_Tree_Atom(
                    "ilst",
                    [M4A_ILST_Leaf_Atom(
                        '\xa9too',
                        [M4A_ILST_Unicode_Data_Atom(
                            0, 1, "Python Audio Tools %s" % (VERSION))])]),
                M4A_FREE_Atom(1024)])

    @classmethod
    def __free_atom__(cls, size):
        return M4A_FREE_Atom(size)
