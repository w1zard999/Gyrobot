import unittest

from wasdcam import FrameParser, looks_like_camera


def part(jpeg):
    return (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")


HDR = b"HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n"


class T(unittest.TestCase):
    def test_whole_frames(self):
        p = FrameParser()
        self.assertEqual(p.feed(HDR + part(b"AAA") + part(b"BBBB")), [b"AAA", b"BBBB"])

    def test_frame_split_across_chunks(self):
        data = HDR + part(b"J" * 5000) + part(b"K" * 10)
        p = FrameParser()
        out = []
        for i in range(0, len(data), 700):          # как приходит из сокета — кусками
            out += p.feed(data[i:i + 700])
        self.assertEqual(out, [b"J" * 5000, b"K" * 10])

    def test_jpeg_may_contain_boundary_text(self):
        tricky = b"\xff\xd8--frame\r\nContent-Length: 3\r\n\r\nxyz\xff\xd9"
        p = FrameParser()
        self.assertEqual(p.feed(HDR + part(tricky) + part(b"Z")), [tricky, b"Z"])

    def test_garbage_before_first_frame_is_skipped(self):
        p = FrameParser()
        self.assertEqual(p.feed(b"\x00\x01junk" + part(b"OK")), [b"OK"])

    def test_bad_length_drops_buffer_instead_of_growing(self):
        p = FrameParser(max_frame=1000)
        p.feed(b"--frame\r\nContent-Length: 99999999\r\n\r\n" + b"x" * 5000)
        self.assertLess(len(p.buf), 1000)
        self.assertEqual(p.feed(part(b"NEXT")), [b"NEXT"])

    def test_looks_like_camera(self):
        self.assertTrue(looks_like_camera(b"HTTP/1.1 200 OK\r\n\r\n<h2>OpenMV Cam - Status</h2>"))
        self.assertFalse(looks_like_camera(b"HTTP/1.0 404 Not Found\r\n\r\n<h1>Not Found</h1>"))
        self.assertFalse(looks_like_camera(b""))


if __name__ == "__main__":
    unittest.main()
