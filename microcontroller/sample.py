import serial
import pygame
import sys

pygame.init()
screen = pygame.display.set_mode((640, 480))
clock = pygame.time.Clock()

teensy = serial.Serial("/dev/ttyACM0", 57600)

def get_data():
  line = teensy.readline()
  print line
  if line.find("TEENSY") == -1:
    return []
  line = line.split(" ")[1:]
  print line
  return [int(line[0]), int(line[1]), float(line[2]), float(line[3])]

def main():
  while True:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        pygame.quit()
        sys.exit()
    screen.fill((0, 0, 0))
    values = get_data()
    print values
    if len(values) != 4:
      continue
    x = values[2] * 10 + 240
    y = values[3] * 10 + 240
    l = values[0]
    r = values[1]
    pygame.draw.line(screen, (255, 0, 0), (240, 240), (x, y))
    pygame.draw.rect(screen, (0, 255, 0), pygame.Rect(480, 240, 80, l))
    pygame.draw.rect(screen, (0, 0, 255), pygame.Rect(560, 240, 80, r))

    pygame.display.flip()
    clock.tick(40)

main()
