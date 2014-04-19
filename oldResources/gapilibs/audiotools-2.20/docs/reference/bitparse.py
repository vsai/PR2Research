#!/usr/bin/python

import sys
import re

try:
    from reportlab.pdfgen import canvas
    from reportlab.lib.units import inch
    from reportlab.pdfbase.pdfmetrics import registerFont
    from reportlab.pdfbase.ttfonts import TTFont
except ImportError:
    print "*** ReportLab is required"
    print "Please fetch the open-source version from http://www.reportlab.org"
    sys.exit(1)

#this size of an individual bit cell, in points
# BIT_WIDTH = 0
BIT_HEIGHT = 30

(BORDER_NONE, BORDER_LINE, BORDER_DOTTED) = range(3)

class RGB_Color:
    RGB = re.compile(r'^#([0-9A-Fa-f]{2})([0-9A-Fa-f]{2})([0-9A-Fa-f]{2})$')
    RGBA = re.compile(r'^#([0-9A-Fa-f]{2})([0-9A-Fa-f]{2})([0-9A-Fa-f]{2})' +
                      r'([0-9A-Fa-f]{2})$')

    COLOR_TABLE = {u"red":(1.0, 0.0, 0.0),
                   u"orange":(1.0, 0.4, 0.0),
                   u"yellow":(1.0, 1.0, 0.0),
                   u"green":(0.0, 1.0, 0.0),
                   u"blue":(0.0, 0.0, 1.0),
                   u"aqua":(0.0, 1.0, 1.0),
                   u"black":(0.0, 0.0, 0.0),
                   u"fuchsia":(1.0, 0.0, 1.0),
                   u"gray":(0.5, 0.5, 0.5),
                   u"lime":(0.0, 1.0, 0.0),
                   u"maroon":(0.5, 0.0, 0.0),
                   u"navy":(0.0, 0.0, 0.5),
                   u"olive":(0.5, 0.5, 0.0),
                   u"purple":(0.5, 0.0, 0.5),
                   u"silver":(0.75, 0.75, 0.75),
                   u"teal":(0.0, 0.5, 0.5),
                   u"white":(1.0, 1.0, 1.0)}

    def __init__(self, red, green, blue, alpha=None):
        """all should be floats between 0.0 and 1.0"""

        self.red = red
        self.green = green
        self.blue = blue
        self.alpha = alpha

    @classmethod
    def from_string(cls, s):
        if (s in cls.COLOR_TABLE):
            (r, g, b) = cls.COLOR_TABLE[s]
            return cls(red=r, green=g, blue=b, alpha=1.0)
        else:
            rgb = cls.RGB.match(s)
            if (rgb is not None):
                return cls(red=int(rgb.group(1), 16) / 255.0,
                           green=int(rgb.group(2), 16) / 255.0,
                           blue=int(rgb.group(3), 16) / 255.0)
            else:
                rgba = cls.RGBA.match(s)
                if (rgba is not None):
                    return cls(red=int(rgba.group(1), 16) / 255.0,
                               green=int(rgba.group(2), 16) / 255.0,
                               blue=int(rgba.group(3), 16) / 255.0,
                               alpha=int(rgba.group(4), 16) / 255.0)
                else:
                    raise ValueError("invalid color string %s" % (repr(s)))


class Chunk:
    def __init__(self, bits, superscripts,
                 name=None,
                 background_color=None,
                 w_border=BORDER_NONE, e_border=BORDER_NONE):
        assert(len(bits) == len(superscripts))
        self.bits = bits
        self.superscripts = superscripts
        self.name = name
        self.background_color = background_color
        self.w_border = w_border
        self.e_border = e_border

        #the chunk's location in the PDF, in x,y point pairs
        self.ne = self.nw = self.se = self.sw = (0, 0)

    def set_w_offset(self, x):
        global BIT_WIDTH

        self.nw = (x, self.nw[1])
        self.sw = (x, self.sw[1])
        self.ne = (x + (self.size() * BIT_WIDTH), self.ne[1])
        self.se = (x + (self.size() * BIT_WIDTH), self.se[1])

    def set_s_offset(self, y):
        self.nw = (self.nw[0], y + BIT_HEIGHT)
        self.sw = (self.sw[0], y)
        self.ne = (self.ne[0], y + BIT_HEIGHT)
        self.se = (self.se[0], y)

    def size(self):
        return len(self.bits)

    def split(self, bits):
        return (Chunk(bits=self.bits[0:bits],
                      superscripts=self.superscripts[0:bits],
                      name=self.name,
                      background_color=self.background_color,
                      w_border=self.w_border,
                      e_border=BORDER_NONE),
                Chunk(bits=self.bits[bits:],
                      superscripts=self.superscripts[bits:],
                      name=self.name,
                      background_color=self.background_color,
                      w_border=BORDER_NONE,
                      e_border=self.e_border))

    def __repr__(self):
        return "Chunk(%s)" % \
            ",".join(["%s=%s" % (attr, repr(getattr(self, attr)))
                      for attr in ["bits", "superscripts", "name",
                                   "background_color",
                                   "w_border", "e_border",
                                   "nw", "ne", "sw", "se"]])

    def pt_width(self):
        return abs(self.nw[0] - self.ne[0])

    def pt_height(self):
        return abs(self.nw[1] - self.sw[1])

    def to_pdf(self, pdf):
        pts_per_bit = self.pt_width() / float(len(self.bits))
        pt_offset = self.nw[0] + (pts_per_bit / 2)

        #draw background color, if any
        if (self.background_color is not None):
            pdf.setFillColorRGB(r=self.background_color.red,
                                g=self.background_color.green,
                                b=self.background_color.blue,
                                alpha=self.background_color.alpha)
            pdf.rect(self.sw[0], self.sw[1], self.pt_width(), self.pt_height(),
                     stroke=0, fill=1)

        pdf.setFillColorRGB(0.0, 0.0, 0.0, 1.0)
        for (i, (bit, superscript)) in enumerate(zip(self.bits,
                                                     self.superscripts)):
            #draw bit
            pdf.setFont("Courier", 18)
            pdf.drawCentredString((i * pts_per_bit) + pt_offset,
                                  self.se[1] + 12, unicode(bit))

            #draw superscript, if any
            if (superscript is not None):
                pdf.setFont("Courier", 5)
                pdf.drawRightString(self.nw[0] + ((i + 1) * pts_per_bit) - 2,
                                    self.se[1] + 25,
                                    unicode(superscript))

        #draw centered name, if any
        if ((self.name is not None) and
            (pdf.stringWidth(unicode(self.name),
                             "DejaVu",
                             6) <= (self.pt_width() * 2))):
            pdf.setFont("DejaVu", 6)
            pdf.drawCentredString(self.nw[0] + (self.pt_width() / 2),
                                  self.se[1] + 2,
                                  unicode(self.name))

        pdf.setStrokeColorRGB(0.0, 0.0, 0.0, 1.0)
        #draw top and bottom borders
        pdf.setDash()
        pdf.line(self.nw[0], self.nw[1],
                 self.ne[0], self.ne[1])
        pdf.line(self.sw[0], self.sw[1],
                 self.se[0], self.se[1])

        #draw left and right borders, if any
        if (self.w_border == BORDER_LINE):
            pdf.setDash()
            pdf.line(self.nw[0], self.nw[1],
                     self.sw[0], self.sw[1])
        elif (self.w_border == BORDER_DOTTED):
            pdf.setDash(1, 6)
            pdf.line(self.nw[0], self.nw[1],
                     self.sw[0], self.sw[1])

        if (self.e_border == BORDER_LINE):
            pdf.setDash()
            pdf.line(self.ne[0], self.ne[1],
                     self.se[0], self.se[1])
        elif (self.e_border == BORDER_DOTTED):
            pdf.setDash(1, 6)
            pdf.line(self.ne[0], self.ne[1],
                     self.se[0], self.se[1])

    def to_svg(self, dom, svg, total_height):
        pts_per_bit = self.pt_width() / float(len(self.bits))
        pt_offset = self.nw[0] + (pts_per_bit / 2)

        #draw background color, if any
        #FIXME

        for (i, (bit, superscript)) in enumerate(zip(self.bits,
                                                     self.superscripts)):
            #draw bit
            bit_xml = dom.createElement(u"text")
            bit_xml.setAttribute(u"style",
                                 u"font-size: 18pt; font-family: Courier")
            bit_xml.setAttribute(u"text-anchor", u"middle")
            bit_xml.setAttribute(u"x",
                                 u"%dpt" % ((i * pts_per_bit) + pt_offset))
            bit_xml.setAttribute(u"y",
                                 u"%dpt" % (total_height -
                                            (self.se[1]) - 12))
            bit_text = dom.createTextNode(unicode(bit))
            bit_xml.appendChild(bit_text)
            svg.appendChild(bit_xml)

            #draw superscript, if any
            if (superscript is not None):
                ss_xml = dom.createElement(u"text")
                ss_xml.setAttribute(u"style",
                                    u"font-size: 5pt; font-family: Courier")
                ss_xml.setAttribute(u"text-anchor", u"end")
                ss_xml.setAttribute(u"x",
                                    u"%dpt" % (self.nw[0] +
                                               ((i + 1) * pts_per_bit) - 2))
                ss_xml.setAttribute(u"y",
                                    u"%dpt" % (total_height -
                                               (self.se[1]) - 25))
                ss_text = dom.createTextNode(unicode(superscript))
                ss_xml.appendChild(ss_text)
                svg.appendChild(ss_xml)

        #draw centered name, if any
        name = dom.createElement(u"text")
        name.setAttribute(u"style", u"font-size: 6pt; font-family: sans-serif")
        name.setAttribute(u"text-anchor", u"middle")
        name.setAttribute(u"x", u"%dpt" % (self.nw[0] + (self.pt_width() / 2)))
        name.setAttribute(u"y", u"%dpt" % (total_height - (self.se[1]) - 2))
        name_text = dom.createTextNode(unicode(self.name))
        name.appendChild(name_text)
        svg.appendChild(name)

        #draw top and bottom borders
        top = dom.createElement(u"line")
        top.setAttribute(u"x1", "%dpt" % (self.nw[0]))
        top.setAttribute(u"y1", "%dpt" % (total_height - self.nw[1]))
        top.setAttribute(u"x2", "%dpt" % (self.ne[0]))
        top.setAttribute(u"y2", "%dpt" % (total_height - self.ne[1]))
        top.setAttribute(u"style", u"stroke: black;")
        svg.appendChild(top)

        bottom = dom.createElement(u"line")
        bottom.setAttribute(u"x1", "%dpt" % (self.sw[0]))
        bottom.setAttribute(u"y1", "%dpt" % (total_height - self.sw[1]))
        bottom.setAttribute(u"x2", "%dpt" % (self.se[0]))
        bottom.setAttribute(u"y2", "%dpt" % (total_height - self.se[1]))
        bottom.setAttribute(u"style", u"stroke: black;")
        svg.appendChild(bottom)

        #drop left and right borders, if any
        #FIXME - handle dotted/line borders
        if (self.w_border):
            left = dom.createElement(u"line")
            left.setAttribute(u"x1", "%dpt" % (self.nw[0]))
            left.setAttribute(u"y1", "%dpt" % (total_height - self.nw[1]))
            left.setAttribute(u"x2", "%dpt" % (self.sw[0]))
            left.setAttribute(u"y2", "%dpt" % (total_height - self.sw[1]))
            left.setAttribute(u"style", u"stroke: black;")
            svg.appendChild(left)

        #FIXME - handle dotted/line borders
        if (self.e_border):
            right = dom.createElement(u"line")
            right.setAttribute(u"x1", "%dpt" % (self.ne[0]))
            right.setAttribute(u"y1", "%dpt" % (total_height - self.ne[1]))
            right.setAttribute(u"x2", "%dpt" % (self.se[0]))
            right.setAttribute(u"y2", "%dpt" % (total_height - self.se[1]))
            right.setAttribute(u"style", u"stroke: black;")
            svg.appendChild(right)


class TextChunk(Chunk):
    def __init__(self, bit_width,
                 name=None,
                 background_color=None,
                 w_border=BORDER_NONE, e_border=BORDER_NONE):
        self.bit_width = bit_width
        self.name = name
        self.background_color = background_color
        self.w_border = w_border
        self.e_border = e_border

        #the chunk's location in the PDF, in x,y point pairs
        self.ne = self.nw = self.se = self.sw = (0, 0)

    def size(self):
        return self.bit_width

    def split(self, bits):
        return (TextChunk(bit_width=bits,
                          name=self.name,
                          background_color=self.background_color,
                          w_border=self.w_border,
                          e_border=BORDER_NONE),
                TextChunk(bit_width=self.bit_width - bits,
                          name=self.name,
                          background_color=self.background_color,
                          w_border=self.w_border,
                          e_border=BORDER_NONE))

    def __repr__(self):
        return "TextChunk(%s)" % \
            ",".join([repr(getattr(self, attr))
                      for attr in ["bit_width", "name",
                                   "background_color",
                                   "w_border", "e_border",
                                   "nw", "ne", "sw", "se"]])

    def to_pdf(self, pdf):
        pts_per_bit = self.pt_width() / float(self.bit_width)
        pt_offset = self.nw[0] + (pts_per_bit / 2)

        #draw background color, if any
        if (self.background_color is not None):
            pdf.setFillColorRGB(r=self.background_color.red,
                                g=self.background_color.green,
                                b=self.background_color.blue,
                                alpha=self.background_color.alpha)
            pdf.rect(self.sw[0], self.sw[1], self.pt_width(), self.pt_height(),
                     stroke=0, fill=1)

        #draw centered name, if any
        pdf.setFillColorRGB(0.0, 0.0, 0.0, 1.0)
        pdf.setFont("DejaVu", 12)
        pdf.drawCentredString(self.nw[0] + (self.pt_width() / 2),
                              self.se[1] + 12,
                              unicode(self.name))

        pdf.setStrokeColorRGB(0.0, 0.0, 0.0, 1.0)
        #drop top and bottom borders
        pdf.setDash()
        pdf.line(self.nw[0], self.nw[1],
                 self.ne[0], self.ne[1])
        pdf.line(self.sw[0], self.sw[1],
                 self.se[0], self.se[1])

        #draw left and right borders, if any
        if (self.w_border == BORDER_LINE):
            pdf.setDash()
            pdf.line(self.nw[0], self.nw[1],
                     self.sw[0], self.sw[1])
        elif (self.w_border == BORDER_DOTTED):
            pdf.setDash(1, 6)
            pdf.line(self.nw[0], self.nw[1],
                     self.sw[0], self.sw[1])

        if (self.e_border == BORDER_LINE):
            pdf.setDash()
            pdf.line(self.ne[0], self.ne[1],
                     self.se[0], self.se[1])
        elif (self.e_border == BORDER_DOTTED):
            pdf.setDash(1, 6)
            pdf.line(self.ne[0], self.ne[1],
                     self.se[0], self.se[1])

    def to_svg(self, dom, svg, total_height):
        pts_per_bit = self.pt_width() / float(self.bit_width)
        pt_offset = self.nw[0] + (pts_per_bit / 2)

        #draw background color, if any
        #FIXME

        #draw centered name, if any
        name = dom.createElement(u"text")
        name.setAttribute(u"style", u"font-size: 18; font-family: sans-serif")
        name.setAttribute(u"text-anchor", u"middle")
        name.setAttribute(u"x", u"%dpt" % (self.nw[0] + (self.pt_width() / 2)))
        name.setAttribute(u"y", u"%dpt" % (total_height -
                                           self.se[1] - 12))
        name_text = dom.createTextNode(unicode(self.name))
        name.appendChild(name_text)
        svg.appendChild(name)

        #drop top and bottom borders
        top = dom.createElement(u"line")
        top.setAttribute(u"x1", "%dpt" % (self.nw[0]))
        top.setAttribute(u"y1", "%dpt" % (total_height - self.nw[1]))
        top.setAttribute(u"x2", "%dpt" % (self.ne[0]))
        top.setAttribute(u"y2", "%dpt" % (total_height - self.ne[1]))
        top.setAttribute(u"style", u"stroke: black;")
        svg.appendChild(top)

        bottom = dom.createElement(u"line")
        bottom.setAttribute(u"x1", "%dpt" % (self.sw[0]))
        bottom.setAttribute(u"y1", "%dpt" % (total_height - self.sw[1]))
        bottom.setAttribute(u"x2", "%dpt" % (self.se[0]))
        bottom.setAttribute(u"y2", "%dpt" % (total_height - self.se[1]))
        bottom.setAttribute(u"style", u"stroke: black;")
        svg.appendChild(bottom)

        #drop left and right borders, if any
        #FIXME - handle dotted/line borders
        if (self.w_border):
            left = dom.createElement(u"line")
            left.setAttribute(u"x1", "%dpt" % (self.nw[0]))
            left.setAttribute(u"y1", "%dpt" % (total_height - self.nw[1]))
            left.setAttribute(u"x2", "%dpt" % (self.sw[0]))
            left.setAttribute(u"y2", "%dpt" % (total_height - self.sw[1]))
            left.setAttribute(u"style", u"stroke: black;")
            svg.appendChild(left)

        #FIXME - handle dotted/line borders
        if (self.e_border):
            right = dom.createElement(u"line")
            right.setAttribute(u"x1", "%dpt" % (self.ne[0]))
            right.setAttribute(u"y1", "%dpt" % (total_height - self.ne[1]))
            right.setAttribute(u"x2", "%dpt" % (self.se[0]))
            right.setAttribute(u"y2", "%dpt" % (total_height - self.se[1]))
            right.setAttribute(u"style", u"stroke: black;")
            svg.appendChild(right)

class BytesChunk(Chunk):
    def __init__(self, bit_width, superscripts,
                 name=None,
                 background_color=None,
                 w_border=BORDER_NONE, e_border=BORDER_NONE):
        assert(bit_width == len(superscripts))
        self.bit_width = bit_width
        self.superscripts = superscripts
        self.name = name
        self.background_color = background_color
        self.w_border = w_border
        self.e_border = e_border

        #the chunk's location in the PDF, in x,y point pairs
        self.ne = self.nw = self.se = self.sw = (0, 0)

    def size(self):
        return self.bit_width

    def split(self, bits):
        return (BytesChunk(bit_width=bits,
                           superscripts=self.superscripts[0:bits],
                           name=self.name,
                           background_color=self.background_color,
                           w_border=self.w_border,
                           e_border=BORDER_NONE),
                BytesChunk(bit_width=self.bit_width - bits,
                           superscripts=self.superscripts[bits:],
                           name=self.name,
                           background_color=self.background_color,
                           w_border=BORDER_NONE,
                           e_border=self.e_border))

    def __repr__(self):
        return "BytesChunk(%s)" % \
            ",".join([repr(getattr(self, attr))
                      for attr in ["bit_width", "superscripts", "name",
                                   "background_color", "w_border", "e_border",
                                   "nw", "ne", "sw", "se"]])

    def to_pdf(self, pdf):
        pts_per_bit = self.pt_width() / float(self.size())
        pt_offset = self.nw[0] + (pts_per_bit / 2)

        #draw background color, if any
        if (self.background_color is not None):
            #FIXME
            raise NotImplementedError()

        pdf.setFillColorRGB(0.0, 0.0, 0.0, 1.0)
        #draw superscripts, if any
        for (i, superscript) in enumerate(self.superscripts):
            pdf.setFont("Courier", 5)
            pdf.drawRightString(self.nw[0] + ((i + 1) * pts_per_bit) - 2,
                                self.se[1] + 25,
                                unicode(superscript))

        #draw centered name, if any
        if (self.name is not None):
            pdf.setFont("DejaVu", 12)
            pdf.drawCentredString(self.nw[0] + (self.pt_width() / 2),
                                  self.se[1] + 12,
                                  unicode(self.name))

        pdf.setStrokeColorRGB(0.0, 0.0, 0.0, 1.0)
        #draw top and bottom borders
        pdf.setDash()
        pdf.line(self.nw[0], self.nw[1],
                 self.ne[0], self.ne[1])
        pdf.line(self.sw[0], self.sw[1],
                 self.se[0], self.se[1])

        #draw left and right borders, if any
        if (self.w_border == BORDER_LINE):
            pdf.setDash()
            pdf.line(self.nw[0], self.nw[1],
                     self.sw[0], self.sw[1])
        elif (self.w_border == BORDER_DOTTED):
            pdf.setDash(1, 6)
            pdf.line(self.nw[0], self.nw[1],
                     self.sw[0], self.sw[1])

        if (self.e_border == BORDER_LINE):
            pdf.setDash()
            pdf.line(self.ne[0], self.ne[1],
                     self.se[0], self.se[1])
        elif (self.e_border == BORDER_DOTTED):
            pdf.setDash(1, 6)
            pdf.line(self.ne[0], self.ne[1],
                     self.se[0], self.se[1])

    def to_svg(self, dom, svg, total_height):
        raise NotImplementedError()


class ChunkTable:
    def __init__(self, chunks):
        self.chunks = chunks

    def to_pdf(self, total_width, filename):
        total_height = max([chunk.nw[1] for chunk in self.chunks])

        registerFont(TTFont("DejaVu", "DejaVuSans.ttf"))

        pdf = canvas.Canvas(filename)
        pdf.setPageSize((total_width,total_height))

        for chunk in self.chunks:
            chunk.to_pdf(pdf)

        pdf.showPage()
        pdf.save()

    def to_svg(self, total_width, filename):
        total_height = max([chunk.nw[1] for chunk in self.chunks])

        import xml.dom.minidom

        impl = xml.dom.minidom.getDOMImplementation()
        dom = impl.createDocument(None, None, None)

        svg = dom.createElement(u"svg")
        svg.setAttribute(u"xmlns",u"http://www.w3.org/2000/svg")
        svg.setAttribute(u"version", u"1.1")
        svg.setAttribute(u"width", u"%dpt" % (total_width))
        svg.setAttribute(u"height", u"%dpt" % (total_height))

        for chunk in self.chunks:
            chunk.to_svg(dom, svg, total_height)

        dom.appendChild(svg)

        f = open(filename, "w")
        dom.writexml(f)
        f.close()


class Bits:
    def __init__(self, name, bits, init, e_border, w_border, background_color):
        """name is a unicode string
        bits is a list of individual bit values
        this generates chunks to be displayed"""

        self.name = name
        self.bits = bits
        self.init = init
        self.e_border = e_border
        self.w_border = w_border
        self.background_color = background_color

    def __repr__(self):
        return "Bits(%s)" % \
            ",".join(["%s=%s" % (attr, repr(getattr(self, attr)))
                      for attr in ["name", "bits", "init",
                                   "e_border", "w_border"]])

    def chunk(self, superscript_bits, bits_lookup):
        if (self.init is not None):
            for i in xrange(len(superscript_bits)):
                superscript_bits.pop(-1)
            superscript_bits.extend(self.init)

        chunk_superscripts = []
        for bit in self.bits:
            superscript_bits.append(bit)
            if (tuple(superscript_bits) in bits_lookup):
                chunk_superscripts.append(bits_lookup[tuple(superscript_bits)])
                for i in xrange(len(superscript_bits)):
                    superscript_bits.pop(-1)
            else:
                chunk_superscripts.append(None)

        return Chunk(bits=self.bits,
                     superscripts=chunk_superscripts,
                     name=self.name,
                     background_color=self.background_color,
                     e_border=self.e_border,
                     w_border=self.w_border)


class Text:
    def __init__(self, name, bit_count, e_border, w_border, background_color):
        self.name = name
        self.bit_count = bit_count
        self.e_border = e_border
        self.w_border = w_border
        self.background_color = background_color

    def __repr__(self):
        return "Text(%s)" % \
            ",".join(["%s=%s" % (attr, repr(getattr(self, attr)))
                      for attr in ["name", "bit_count",
                                   "e_border", "w_border",
                                   "background_color"]])

    def chunk(self, superscript_bits, bits_lookup):
        for i in xrange(len(superscript_bits)):
            superscript_bits.pop(-1)

        return TextChunk(bit_width=self.bit_count,
                         name=self.name,
                         background_color=self.background_color,
                         e_border=self.e_border,
                         w_border=self.w_border)

class Bytes:
    def __init__(self, name, bytes_list, e_border, w_border, background_color):
        self.name = name
        self.bytes = list(bytes_list)
        self.e_border = e_border
        self.w_border = w_border

    def __repr__(self):
        return "Text(%s)" % \
            ",".join(["%s=%s" % (attr, repr(getattr(self, attr)))
                      for attr in ["name", "bytes",
                                   "e_border", "w_border"]])


    def chunk(self, superscript_bits, bits_lookup):
        reverse_bits_lookup = dict([(value, key) for (key, value) in
                                    bits_lookup.items()])
        superscript_len = list(set(map(len, bits_lookup.keys())))[0]

        if (len(superscript_bits) != 0):
            print >>sys.stderr,"*** Warning: <bytes> tag %s not byte-aligned" % (repr(self.name))

        chunk_superscripts = []
        for byte in self.bytes:
            superscript_bits.extend(
                list(reverse_bits_lookup["%2.2X" % (byte)]))
            chunk_superscripts.append(
                bits_lookup[tuple(superscript_bits[0:superscript_len])])
            for i in xrange(superscript_len):
                superscript_bits.pop(0)

        return BytesChunk(bit_width=len(self.bytes),
                          superscripts=chunk_superscripts,
                          name=self.name,
                          e_border=self.e_border,
                          w_border=self.w_border)

def bits(v):
    for i in xrange(8):
        yield v & 1
        v >>= 1


BE_LOOKUP = dict([(tuple(reversed(list(bits(value)))), "%2.2X" % (value))
                  for value in xrange(0,0x100)])

LE_LOOKUP = dict([(tuple(bits(value)), "%2.2X" % (value))
                  for value in xrange(0,0x100)])


def bits_to_chunks(bits_iter, lookup=BE_LOOKUP, initial_superscript_bits=None):
    """for each Bits object in bits_iter, yields a Chunk object
    whose bits, superscripts, name and background_color have been populated

    positions and borders must be populated afterward
    """

    if (initial_superscript_bits is None):
        superscript_bits = []
    else:
        superscript_bits = initial_superscript_bits
    for bits in bits_iter:
        yield bits.chunk(superscript_bits, lookup)


def chunks_to_rows(chunks_iter, bits_per_row, x_offset=0):
    """for each Chunk object in chunks_iter
    yields a list of Chunk objects
    whose borders and east/west positions have been populated"""

    chunk_list = []
    x_position = x_offset

    for chunk in chunks_iter:
        remaining_bits = bits_per_row - sum([c.size() for c in chunk_list])

        #split a single chunk across multiple rows, if necessary
        while (chunk.size() > remaining_bits):
            if (remaining_bits > 0):
                (row_end, row_start) = chunk.split(remaining_bits)

                #populate row_end's east/west positions
                row_end.set_w_offset(x_position)

                #before appending it to the row's chunks for returning
                chunk_list.append(row_end)
                yield chunk_list
                chunk = row_start
            elif (len(chunk_list) > 0):
                yield chunk_list

            #and resetting the row for the remainder of the chunk
            chunk_list = []
            x_position = x_offset
            remaining_bits = bits_per_row
        else:
            global BIT_WIDTH

            #populate the chunk's east/west positions
            chunk.set_w_offset(x_position)

            #and update the east/west position
            x_position += chunk.size() * BIT_WIDTH

            #before appending it to the row's chunks for returning
            chunk_list.append(chunk)
    else:
        #return any leftover chunks on the row
        yield chunk_list


def align_rows(chunk_rows_iter):
    chunk_rows = list(chunk_rows_iter)

    for (i, chunk_row) in enumerate(reversed(chunk_rows)):
        for chunk in chunk_row:
            chunk.set_s_offset(i * BIT_HEIGHT)

    for chunk_row in chunk_rows:
        for chunk in chunk_row:
            yield chunk


def int_converter(u):
    if (u.startswith(u"0x")):
        return long(u[2:], 16)
    elif (u.endswith(u"b")):
        return long(u[0:-1], 2)
    else:
        return long(u)


def bits_converter_be(size, value):
    size = int_converter(size)
    value = int_converter(value)

    bits = []
    for i in xrange(size):
        bits.append(value & 1)
        value >>= 1

    bits.reverse()

    return bits


def bits_converter_le(size, value):
    size = int_converter(size)
    value = int_converter(value)

    bits = []
    for i in xrange(size):
        bits.append(value & 1)
        value >>= 1

    return bits

def byte_converter(value):
    if (not re.match(r'^[0-9a-fA-F]+$', value)):
        raise ValueError("bytes value must be hexadecimal")
    elif ((len(value) % 2) != 0):
        raise ValueError("bytes value must be an even number of digits")
    elif (len(value) == 0):
        raise ValueError("at least 1 byte must be present")
    else:
        while (len(value) > 0):
            yield int(value[0:2], 16)
            value = value[2:]

def init_converter(node):
    if (node.hasAttribute(u"init")):
        lookup = {u"0":0, u"1":1}
        return [lookup[char] for char in
                node.getAttribute(u'init').split(u",")]
    else:
        return None

def get_border(node, attrib):
    if (node.hasAttribute(attrib)):
        style = node.getAttribute(attrib)
        if (style == u'blank'):
            return BORDER_NONE
        elif (style == u'line'):
            return BORDER_LINE
        elif (style == u'dotted'):
            return BORDER_DOTTED
        else:
            raise ValueError(u"unknown border style %s" % (style))
    else:
        return BORDER_LINE

def xml_to_chunks(xml_filename):
    import xml.dom.minidom

    dom = xml.dom.minidom.parse(xml_filename)
    struct = dom.getElementsByTagName(u"struct")[0]

    if (not struct.hasAttribute(u'endianness')):
        print >>sys.stderr,"struct tag's endianness must be big or little"
        sys.exit(1)

    if (struct.hasAttribute(u'init')):
        lookup = {u"0":0, u"1":1}
        superscript_bits = [lookup[char] for char in
                            struct.getAttribute(u'init').split(u",")]
    else:
        superscript_bits = None

    if (struct.getAttribute(u'endianness') == u'big'):
        lookup = BE_LOOKUP
        bits_converter = bits_converter_be
    elif (struct.getAttribute(u'endianness') == u'little'):
        lookup = LE_LOOKUP
        bits_converter = bits_converter_le
    else:
        print >>sys.stderr,"struct tag's endianness must be big or little"
        sys.exit(1)

    bits = []
    for part in struct.childNodes:
        if ((part.nodeName in (u"field", u"text", u"bytes")) and
            part.hasAttribute(u"background-color")):
            background_color = RGB_Color.from_string(
                part.getAttribute(u"background-color"))
        else:
            background_color = None

        if (part.nodeName == u'field'):
            bits.append(Bits(
                    name=(part.childNodes[0].data.strip()
                          if len(part.childNodes) else u""),
                    bits=bits_converter(part.getAttribute(u"size"),
                                        part.getAttribute(u"value")),
                    init=init_converter(part),
                    e_border=get_border(part, u"border_e"),
                    w_border=get_border(part, u"border_w"),
                    background_color=background_color))
        elif (part.nodeName == u'text'):
            bits.append(Text(
                    name=(part.childNodes[0].data.strip()
                          if len(part.childNodes) else u""),
                    bit_count=int_converter(part.getAttribute(u"size")),
                    e_border=get_border(part, u"border_e"),
                    w_border=get_border(part, u"border_w"),
                    background_color=background_color))
        elif (part.nodeName == u'bytes'):
            try:
                bits.append(Bytes(
                        name=(part.childNodes[0].data.strip()
                              if len(part.childNodes) else u""),
                        bytes_list=byte_converter(part.getAttribute(u"value")),
                        e_border=get_border(part, u"border_e"),
                        w_border=get_border(part, u"border_w"),
                        background_color=background_color))
            except ValueError,msg:
                print >>sys.stderr,str(msg)
                sys.exit(1)

    return bits_to_chunks(bits, lookup, superscript_bits)


if (__name__ == '__main__'):
    import optparse

    parser = optparse.OptionParser()
    parser.add_option('-i','--input',dest='input',help='input XML file')
    parser.add_option('-o','--output',dest='output',help='output file')
    parser.add_option('-b', '--bits-per-row', dest='bits_per_row',
                      type='int', default=16)
    parser.add_option('-w','--width',dest='width',
                      type='int', default=6 * 72,
                      help='diagram width, in PostScript points')
    parser.add_option('-t','--type',dest='type',
                      choices=("pdf", "svg"),
                      help="type of output file",
                      default="pdf")
    parser.add_option('--bit-width', dest='bit_width',
                      type='int', default=20,
                      help='width of each bit value')

    (options,args) = parser.parse_args()

    global BIT_WIDTH
    BIT_WIDTH = options.bit_width

    x_offset = (options.width - (options.bits_per_row * BIT_WIDTH)) / 2

    table = ChunkTable(list(align_rows(chunks_to_rows(
                    chunks_iter=xml_to_chunks(options.input),
                    bits_per_row=options.bits_per_row,
                    x_offset=x_offset))))

    if (options.type == 'pdf'):
        table.to_pdf(options.width, options.output)
    elif (options.type == 'svg'):
        table.to_svg(options.width, options.output)
    else:
        print >>sys.stderr,"unknown output type"
        sys.exit(1)
