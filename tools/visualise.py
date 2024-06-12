# Simple visualisation for Lidar data
# Copyright 2024 Brian Starkey <stark3y@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

import argparse
import math
import pygame
import serial

from collections import deque

def read_points(port, point_queue, max_points=250):
    """read_points reads max_points points from the sensor

    The points are added to point_queue as cartesian coordinates, with angle
    "0" being treated as being in the negative-Y (up) direction.
    """

    while max_points > 0:
        line = port.readline().decode("ascii").strip()

        try:
            angle, distance = line.split(", ")
            angle = float(angle) - 90
            distance = int(distance)
        except ValueError:
            continue

        rads = math.radians(angle)
        point_queue.appendleft((
            distance * math.cos(rads),
            distance * math.sin(rads),
        ))

        max_points -= 1

def get_max_xy(point_queue):
    return abs(max(max(point_queue, key=lambda t: abs(max(t, key=abs))), key=abs))

def draw_points(surface, centre, scale, point_queue):
    alpha = 255
    alpha_step = 255 / (len(point_queue) - 1)

    for point in point_queue:
        end = centre + pygame.math.Vector2(point[0] * scale, point[1] * scale)

        pygame.draw.line(surface, (0, 0, 255, int(alpha * 0.1)), centre, end, 1)
        pygame.draw.circle(surface, pygame.Color(0, 100, 255, int(alpha)), end, 1)

        alpha -= alpha_step

def draw_ring(surface, centre, scale, distance):
    distance_px = distance * scale
    pygame.draw.circle(surface, "white", centre, distance_px, 1)
    pygame.draw.line(surface, "white", centre, (centre[0] + distance_px, centre[1]), 1)

def parse_args():
    parser = argparse.ArgumentParser(prog="visualise", description="Simple Lidar visualisation")
    parser.add_argument("--port", "-p", help="Serial port for Lidar")
    parser.add_argument("--npoints", "-n", help="Number of points of history", type=int, default=1000)

    return parser.parse_args()

def main():
    args = parse_args()

    port = serial.Serial(args.port)
    point_queue = deque([], maxlen=args.npoints)

    pygame.init()
    pygame.font.init()
    font = pygame.font.SysFont(pygame.font.get_default_font(), 18)
    display_info = pygame.display.Info()

    size = min(display_info.current_w, display_info.current_h) - 80;

    screen = pygame.display.set_mode((size, size))
    pygame.display.set_caption("Lidar Visualisation")
    clock = pygame.time.Clock()

    surface = pygame.Surface([size, size], pygame.SRCALPHA)
    centre = pygame.math.Vector2(size // 2, size // 2)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if chr(event.key) == 'q':
                    running = False

        # Get points from sensor
        read_points(port, point_queue)

        max_xy = get_max_xy(point_queue)

        # Calculate scaling
        scale = (size * 0.45) / max_xy

        # Draw points surface
        surface.fill("black")
        draw_points(surface, centre, scale, point_queue)
        draw_ring(surface, centre, scale, max_xy)
        screen.blit(surface, (0, 0)) 

        # Draw text
        text_surface = font.render(f"{int(max_xy)} mm", False, "white")
        text_x = centre[0] + max_xy * scale - text_surface.get_width() - 10
        screen.blit(text_surface, (text_x, centre[1])) 

        pygame.display.flip()

        clock.tick(20)

    pygame.quit()

if __name__ == "__main__":
    main()
