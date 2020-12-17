import cv2
import pygame, pygame.camera

pygame.init()
pygame.camera.init()
print(pygame.camera.list_cameras())
# cam = pygame.camera.Camera('/dev/video0', (300, 300), 'RGB')
# cam.start()
