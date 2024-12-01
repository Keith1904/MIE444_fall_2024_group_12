import numpy as np
import pygame
import shapely as shp
import settings as SETTINGS
import utilities
import sys

class Maze:
    '''This class represents the maze/environment'''
    def __init__(self):

        self.size_y = SETTINGS.maze_dim_y
        self.size_x = SETTINGS.maze_dim_x

        self.wall_squares = []
        self.walls = []
        self.reduced_walls = []
        self.floor_tiles = []
        self.floor_tile_colors = 0
        self.floor_rect_black = []
        self.floor_rect_white = []
        self.floor_white_poly = 0
        self.orig_walls = SETTINGS.walls
        self.CANVAS_WIDTH = 0
        self.CANVAS_HEIGHT = 0
        

    def import_walls(self):
        '''Imports the walls from a csv file and sets up lines representing them'''

        wall_map = np.array(SETTINGS.walls)
        dim_y = np.size(wall_map, 0)
        dim_x = np.size(wall_map, 1)

        # Outer maze dimensions
        self.wall_squares.append([
            [[0, 0], [dim_x, 0]],
            [[dim_x, 0], [dim_x, dim_y]],
            [[dim_x, dim_y], [0, dim_y]],
            [[0, dim_y], [0, 0]]
            ])

        for ct_x in range(0, dim_x):
            for ct_y in range(0, dim_y):
                if wall_map[ct_y, ct_x] == 0:
                    self.wall_squares.append([
                        [[ct_x, ct_y], [ct_x+1, ct_y]],
                        [[ct_x+1, ct_y], [ct_x+1, ct_y+1]],
                        [[ct_x+1, ct_y+1], [ct_x, ct_y+1]],
                        [[ct_x, ct_y+1], [ct_x, ct_y]]
                        ])

        # Convert to length
        self.wall_squares = [[[[scalar * SETTINGS.wall_segment_length for scalar in point]
                               for point in line]
                              for line in square]
                             for square in self.wall_squares]

        # Flattens list of walls, removes unnecessary walls
        self.walls= [wall for wallsquare in self.wall_squares for wall in wallsquare]
        self.reduced_walls = utilities.optimize_walls(self.walls)

    def draw_walls(self, canvas):
        '''Draws the maze walls onto the screen'''

        # Graphics
        THICKNESS = int(SETTINGS.wall_thickness * SETTINGS.ppi)
        COLOR = SETTINGS.wall_color

        for line in self.reduced_walls:
                start = [scalar * SETTINGS.ppi + SETTINGS.border_pixels for scalar in line[0]]
                end = [scalar * SETTINGS.ppi + SETTINGS.border_pixels for scalar in line[1]]
                pygame.draw.line(canvas, COLOR, start, end, THICKNESS)

    def generate_floor(self):
        '''Generates the floor of the maze'''

        if not self.reduced_walls:
            sys.exit('Walls must be imported before a floor pattern is generated.')

        # Get the number of floor checker points
        dim_x = int(self.size_x / SETTINGS.floor_segment_length)
        dim_y = int(self.size_y / SETTINGS.floor_segment_length)

        # Create the floor tiles
        self.floor_tile_colors = np.floor(np.random.random(dim_y * dim_x)*2) * 255

        floor_tiles = []
        for ct_x in range(0, dim_x):
            for ct_y in range(0, dim_y):
                floor_tiles.append([
                    [ct_x, ct_y], [ct_x+1, ct_y], [ct_x+1, ct_y+1], [ct_x, ct_y+1]
                    ])

        # Convert to length
        self.floor_tiles = [[[scalar * SETTINGS.floor_segment_length for scalar in point]
                             for point in tile]
                            for tile in floor_tiles]

        # Create an array of rectangle objects for drawing
        width = SETTINGS.floor_segment_length * SETTINGS.ppi
        floor_white_polys = []
        for (tile, color) in zip(self.floor_tiles, self.floor_tile_colors.tolist()):
            left = tile[0][0] * SETTINGS.ppi + SETTINGS.border_pixels
            top = tile[0][1] * SETTINGS.ppi + SETTINGS.border_pixels
            tile_rect = pygame.Rect(left, top, width, width)

            # Append the rectangles to floor objects. Create shapely polygons.
            if color:
                self.floor_rect_white.append(tile_rect)
                floor_white_polys.append(shp.Polygon(tile))
            else:
                self.floor_rect_black.append(tile_rect)

        # Create a multipolygon object
        self.floor_white_poly = shp.MultiPolygon(floor_white_polys)

        return True

    def draw_floor(self, canvas):
        '''Draws the maze floor'''
        for white_tile in self.floor_rect_white:
            pygame.draw.rect(canvas, (255, 255, 255), white_tile)
        for black_tile in self.floor_rect_black:
            pygame.draw.rect(canvas, (0, 0, 0), black_tile)