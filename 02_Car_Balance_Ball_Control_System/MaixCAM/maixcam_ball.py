from maix import app, camera, display, image, uart

# ---------- Must be calibrated on the real tube/camera ----------
# LAB threshold of the ball. This example starts with a dark-ball threshold.
BALL_LAB = [[0, 35, -20, 20, -20, 20]]
ROI = [20, 70, 280, 110]

# Pixel centers when the ball is exactly at the -5 cm and +5 cm marks.
# Swapping these two points reverses the reported position sign.
POINT_NEG_5CM = [70.0, 120.0]
POINT_POS_5CM = [250.0, 120.0]

MIN_PIXELS = 40
MIN_ROUNDNESS = 0.45
SERIAL_DEVICE = "/dev/ttyS1"
SERIAL_BAUD = 115200


def send_position(serial, position_mm):
    serial.write(f"{position_mm}\r\n".encode())


def pixel_to_mm(cx, cy):
    vx = POINT_POS_5CM[0] - POINT_NEG_5CM[0]
    vy = POINT_POS_5CM[1] - POINT_NEG_5CM[1]
    mx = 0.5 * (POINT_POS_5CM[0] + POINT_NEG_5CM[0])
    my = 0.5 * (POINT_POS_5CM[1] + POINT_NEG_5CM[1])
    length2 = vx * vx + vy * vy
    if length2 < 1.0:
        return 0.0
    return ((cx - mx) * vx + (cy - my) * vy) * 100.0 / length2


def choose_ball(blobs):
    best = None
    best_score = 0.0
    for blob in blobs:
        width = blob.w()
        height = blob.h()
        if width < 4 or height < 4 or width > 60 or height > 60:
            continue
        ratio = float(width) / float(height)
        roundness = blob.roundness()
        if ratio < 0.55 or ratio > 1.80 or roundness < MIN_ROUNDNESS:
            continue
        score = float(blob.pixels()) * roundness
        if score > best_score:
            best = blob
            best_score = score
    return best


cam = camera.Camera(320, 240)
disp = display.Display()
serial = uart.UART(SERIAL_DEVICE, SERIAL_BAUD)

while not app.need_exit():
    img = cam.read()
    blobs = img.find_blobs(BALL_LAB, roi=ROI,
                           pixels_threshold=MIN_PIXELS,
                           area_threshold=MIN_PIXELS,
                           merge=True, margin=3)
    ball = choose_ball(blobs)
    if ball is None:
        img.draw_string(5, 5, "BALL LOST", color=image.COLOR_RED)
    else:
        position_mm = pixel_to_mm(ball.cxf(), ball.cyf())
        send_position(serial, position_mm)
        img.draw_rect(ball.x(), ball.y(), ball.w(), ball.h(),
                      color=image.COLOR_GREEN)
        img.draw_string(5, 5, "POS %.1f mm" % position_mm,
                        color=image.COLOR_GREEN)
    img.draw_rect(ROI[0], ROI[1], ROI[2], ROI[3], color=image.COLOR_BLUE)
    disp.show(img)
