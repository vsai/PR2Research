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


class DiscID:
    def __init__(self, first_track_number, last_track_number,
                 lead_out_offset, offsets):
        """first_track_number and last_track_number are ints
        typically starting from 1
        lead_out_offset is an integer number of CD frames
        offsets is a list of track offsets, in CD frames"""

        assert((last_track_number - first_track_number + 1) == len(offsets))

        self.first_track_number = first_track_number
        self.last_track_number = last_track_number
        self.lead_out_offset = lead_out_offset
        self.offsets = offsets

    def __repr__(self):
        return "DiscID(%s, %s, %s, %s)" % \
            (self.first_track_number,
             self.last_track_number,
             self.lead_out_offset,
             self.offsets)

    def __str__(self):
        from hashlib import sha1
        from base64 import b64encode

        return b64encode(
            sha1("%2.2X%2.2X%s" %
                 (self.first_track_number,
                  self.last_track_number,
                  "".join(["%8.8X" % (offset) for offset in
                           [self.lead_out_offset] +
                           self.offsets +
                           [0] * (99 - len(self.offsets))
                           ]))).digest(),
            "._").replace("=", "-")

    def __unicode__(self):
        return str(self).decode('ascii')


def perform_lookup(first_track_number, last_track_number,
                   lead_out_offset, offsets,
                   musicbrainz_server, musicbrainz_port):
    """performs a web-based lookup using
    first_track_number, last_track_number, lead_out_offset
    and a list of offset ints

    iterates over a list of MetaData objects per successful match, like:
    [track1, track2, ...], [track1, track2, ...], ...

    may raise urllib2.HTTPError if an error occurs querying the server
    or xml.parsers.expat.ExpatError if there's an error parsing the data
    """

    from urllib2 import urlopen
    from urllib import urlencode
    import xml.dom.minidom
    from itertools import izip

    #build DiscID from input parameters
    disc_id = DiscID(first_track_number,
                     last_track_number,
                     lead_out_offset,
                     offsets)

    #query MusicBrainz web service (version 2) for <metadata>
    m = urlopen("http://%s:%d/ws/2/discid/%s?%s" %
                (musicbrainz_server,
                 musicbrainz_port,
                 disc_id,
                 urlencode({"inc": "artists labels recordings"})))

    xml = xml.dom.minidom.parse(m)

    #for each <release>s in <release-list>
    #yield a list of MetaData objects
    try:
        release_list = get_node(xml, u"metadata", u"disc", u"release-list")
        for release in get_nodes(release_list, u"release"):
            yield list(parse_release(release, disc_id))
    except KeyError:
        #no releases found, so return nothing
        return


def get_node(parent, *nodes):
    """given a minidom tree element and a list of node unicode strings
    indicating a path, returns the node at the end of that path
    or raises KeyError if any node in the path cannot be found"""

    if (len(nodes) == 0):
        return parent
    else:
        for child in parent.childNodes:
            if (hasattr(child, "tagName") and (child.tagName == nodes[0])):
                return get_node(child, *nodes[1:])
        else:
            raise KeyError(nodes[0])


def get_nodes(parent, node):
    """given a minidom tree element and a tag name unicode string,
    returns all the child nodes with that name"""

    return [child for child in parent.childNodes
            if (hasattr(child, "tagName") and (child.tagName == node))]


def text(node):
    """given a minidom leaf node element,
    returns its data as a unicode string"""

    if (node.firstChild is not None):
        return node.firstChild.data
    else:
        return u""


def artist(artist_credit):
    """given an <artist-credit> DOM element,
    returns the artist as a unicode string"""

    artists = []
    #<artist-credit> must contain at least one <name-credit>
    for name_credit in get_nodes(artist_credit, u"name-credit"):
        try:
            #<name-credit> must contain <artist>
            #but <artist> need not contain <name>
            artists.append(text(get_node(name_credit, u"artist", u"name")))
        except KeyError:
            artists.append(u"")
        #<name-credit> may contain a "joinphrase" attribute
        if (name_credit.hasAttribute(u"joinphrase")):
            artists.append(name_credit.getAttribute(u"joinphrase"))
    return u"".join(artists)


def parse_release(release, disc_id):
    """given a <release> Element node and DiscID object
    yields a populated MetaData object per track
    may raise KeyError if the given DiscID is not found in the <release>"""

    #<release> may contain <title>
    try:
        album_name = text(get_node(release, u"title"))
    except KeyError:
        album_name = None

    #<release> may contain <artist-credit>
    try:
        album_artist = artist(get_node(release, u"artist-credit"))
    except KeyError:
        album_artist = None

    #<release> may contain <label-info-list>
    try:
        #<label-info-list> contains 0 or more <label-info>s
        for label_info in get_nodes(get_node(release, u"label-info-list"),
                                    u"label-info"):
            #<label-info> may contain <catalog-number>
            try:
                catalog = text(get_node(label_info, u"catalog-number"))
            except KeyError:
                catalog = None
            #<label-info> may contain <label>
            #and <label> may contain <name>
            try:
                publisher = text(get_node(label_info, u"label", u"name"))
            except KeyError:
                publisher = None

            #we'll use the first result found
            break
        else:
            #<label-info-list> with no <label-info> tags
            catalog = None
            publisher = None
    except KeyError:
        catalog = None
        publisher = None

    #<release> may contain <date>
    try:
        year = text(get_node(release, u"date"))[0:4]
    except:
        year = None

    #find exact disc in <medium-list> tag
    #depending on disc_id value
    try:
        medium_list = get_node(release, u"medium-list")
    except KeyError:
        #no media found for disc ID
        raise KeyError(disc_id)

    for medium in get_nodes(medium_list, u"medium"):
        try:
            if (unicode(disc_id) in
                [disc.getAttribute(u"id")
                 for disc in get_nodes(get_node(medium, u"disc-list"),
                                       u"disc")]):
                #found requested disc_id in <medium>'s list of <disc>s
                #so use that medium node to find additional info
                break
        except KeyError:
            #no <disc-list> tag found in <medium>
            continue
    else:
        #our disc_id wasn't found in any of the <release>'s <medium>s
        raise KeyError(disc_id)

    #if multiple discs in <medium-list>,
    #populate album number and album total
    if ((medium_list.hasAttribute(u"count") and
         (int(medium_list.getAttribute(u"count")) > 1))):
        album_total = int(medium_list.getAttribute(u"count"))
        try:
            album_number = int(text(get_node(medium, u"position")))
        except KeyError:
            album_number = None
    else:
        album_total = album_number = None

    #<medium> must contain <track-list>
    tracks = get_nodes(get_node(medium, u"track-list"), u"track")
    track_total = len(tracks)
    #and <track-list> contains 0 or more <track>s
    for (i, track) in enumerate(tracks):
        #if <track> contains title use that for track_name
        try:
            track_name = text(get_node(track, u"title"))
        except KeyError:
            track_name = None

        #if <track> contains <artist-credit> use that for track_artist
        try:
            track_artist = artist(get_node(release, u"artist-credit"))
        except KeyError:
            track_artist = None

        #if <track> contains a <recording>
        #use that for track_name and track artist
        try:
            recording = get_node(track, u"recording")

            #<recording> may contain a <title>
            if (track_name is None):
                try:
                    track_name = text(get_node(recording, u"title"))
                except KeyError:
                    track_name = None

            #<recording> may contain <artist-credit>
            if (track_artist is None):
                try:
                    track_artist = artist(get_node(recording,
                                                   u"artist-credit"))
                except KeyError:
                    track_artist = album_artist
        except KeyError:
            #no <recording> in <track>

            if (track_artist is None):
                track_artist = album_artist

        #<track> may contain a <position>
        try:
            track_number = int(text(get_node(track, u"position")))
        except KeyError:
            track_number = i + 1

        #yield complete MetaData object
        from . import MetaData

        yield MetaData(track_name=track_name,
                       track_number=track_number,
                       track_total=track_total,
                       album_name=album_name,
                       artist_name=track_artist,
                       performer_name=None,
                       composer_name=None,
                       conductor_name=None,
                       ISRC=None,
                       catalog=catalog,
                       copyright=None,
                       publisher=publisher,
                       year=year,
                       album_number=album_number,
                       album_total=album_total,
                       comment=None)
