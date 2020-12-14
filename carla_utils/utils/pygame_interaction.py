'''
Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
'''

import carla
from carla import ColorConverter as cc

import collections
import datetime
import numpy as np
import random
import re
import weakref
import copy

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_g
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_l
    from pygame.locals import K_i
    from pygame.locals import K_z
    from pygame.locals import K_x
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')



# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def make_surface(carla_image):
    carla_image.convert(cc.Raw)
    array = np.frombuffer(carla_image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (carla_image.height, carla_image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return pygame.surfarray.make_surface(array.swapaxes(0, 1))

def parse_collision_history(history):
    history_dict = collections.defaultdict(int)
    if history:
        for frame, data, intensity in history:
            history_dict[frame] += intensity
    return history_dict



class KeyboardControl(object):
    def __init__(self, hud, display):
        self.hud = hud
        self._autopilot_enabled = False

        self._control = carla.VehicleControl()
        self._lights = carla.VehicleLightState.NONE
        self.hud.vehicle.set_autopilot(self._autopilot_enabled)
        self.hud.vehicle.set_light_state(self._lights)
        self._steer_cache = 0.0

        self._desired_aspect_ratio = self.hud.desired_aspect_ratio


    def parse_events(self, display, clock):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt('quit')
            elif event.type == pygame.VIDEORESIZE:
                width = event.w
                height = event.h
                dim1 = (width, round(width/self._desired_aspect_ratio))
                dim2 = (round(height*self._desired_aspect_ratio), height)
                self._screen_size = min(dim1, dim2)
                # display = pygame.display.set_mode(self.hud.dim, pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE)

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    raise KeyboardInterrupt('quit')
                elif event.key == K_F1:
                    self.hud.toggle_info()
                elif event.key == K_TAB:
                    self.hud.sensors_master.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    self.hud.next_weather(reverse=True)
                elif event.key == K_c:
                    self.hud.next_weather()
                elif event.key == K_BACKQUOTE:
                    pass
                elif event.key == K_n:
                    pass
                elif event.key > K_0 and event.key <= K_9:
                    pass

                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    pass
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    pass
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    pass
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    pass
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    pass

                if event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_m:
                    self._control.manual_gear_shift = not self._control.manual_gear_shift
                    self._control.gear = self.hud.vehicle.get_control().gear
                    self.hud.notification('%s Transmission' %
                                           ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                elif self._control.manual_gear_shift and event.key == K_COMMA:
                    self._control.gear = max(-1, self._control.gear - 1)
                elif self._control.manual_gear_shift and event.key == K_PERIOD:
                    self._control.gear = min(5, self._control.gear + 1)
                elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                    self._autopilot_enabled = not self._autopilot_enabled
                    self.hud.vehicle.set_autopilot(self._autopilot_enabled)
                    self.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                    current_lights ^= carla.VehicleLightState.Special1
                elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                    current_lights ^= carla.VehicleLightState.HighBeam
                elif event.key == K_l:
                    # Use 'L' key to switch between lights:
                    # closed -> position -> low beam -> fog
                    if not self._lights & carla.VehicleLightState.Position:
                        self.hud.notification("Position lights")
                        current_lights |= carla.VehicleLightState.Position
                    else:
                        self.hud.notification("Low beam lights")
                        current_lights |= carla.VehicleLightState.LowBeam
                    if self._lights & carla.VehicleLightState.LowBeam:
                        self.hud.notification("Fog lights")
                        current_lights |= carla.VehicleLightState.Fog
                    if self._lights & carla.VehicleLightState.Fog:
                        self.hud.notification("Lights off")
                        current_lights ^= carla.VehicleLightState.Position
                        current_lights ^= carla.VehicleLightState.LowBeam
                        current_lights ^= carla.VehicleLightState.Fog
                elif event.key == K_i:
                    current_lights ^= carla.VehicleLightState.Interior
                elif event.key == K_z:
                    current_lights ^= carla.VehicleLightState.LeftBlinker
                elif event.key == K_x:
                    current_lights ^= carla.VehicleLightState.RightBlinker

        if not self._autopilot_enabled:
            pressed = self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            self._control.reverse = self._control.gear < 0
            # Set automatic control-related vehicle lights
            if self._control.brake:
                current_lights |= carla.VehicleLightState.Brake
            else: # Remove the Brake flag
                current_lights &= ~carla.VehicleLightState.Brake
            if self._control.reverse:
                current_lights |= carla.VehicleLightState.Reverse
            else: # Remove the Reverse flag
                current_lights &= ~carla.VehicleLightState.Reverse
            if current_lights != self._lights: # Change the light state only if necessary
                self._lights = current_lights
                self.hud.vehicle.set_light_state(carla.VehicleLightState(self._lights))

            if pressed:
                self.hud.vehicle.apply_control(self._control)


    def _parse_vehicle_keys(self, keys, milliseconds):
        flag = False
        if keys[K_UP] or keys[K_w]:
            flag = True
            self._control.throttle = min(self._control.throttle + 0.01, 1)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            flag = True
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            flag = True
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            flag = True
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]
        return flag

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)



class HUD(object):
    def __init__(self, client, world, vehicle, sensors_master, display):
        self.client, self.world, self.town_map = client, world, world.get_map()
        self.vehicle, self.sensors_master = vehicle, sensors_master
        self.display = display

        dim = (self.display.get_width(), self.display.get_height())
        self.dim = dim
        self.origin_dim = dim
        width, height = dim[0], dim[1]
        self.desired_aspect_ratio = float(width) / float(height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))

        self.settings = world.get_settings()
        self.world.on_tick(self.on_world_tick)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._server_clock = pygame.time.Clock()

        self._show_info = True
        self._info_text = []

        self._weather_presets = find_weather_presets()
        self._weather_index = 0


    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, clock):
        self._notifications.tick(clock)
        carla_image = self.sensors_master.get_camera().get_raw_data()
        if carla_image is None: print('[pygame_interaction] tick: warning'); return
        self._surface = make_surface(carla_image)
        if not self._show_info:
            self._info_text = []
            return
        t = self.vehicle.get_transform()
        v = self.vehicle.get_velocity()
        c = self.vehicle.get_control()
        a = self.vehicle.get_acceleration()

        # self.notification('Collision with %r' % data.other_actor.type_id)
        colhist = parse_collision_history(self.sensors_master[('sensor.other.collision', 'default')].get_data())
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = self.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(self.vehicle, truncate=20),
            'Map:     % 20s' % self.town_map.name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * np.sqrt(v.x**2 + v.y**2 + v.z**2)),
            'Accelero: %14.1f m/s2' % (np.sqrt(a.x**2 + a.y**2 + a.z**2)),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        self._info_text += [
            ('Throttle:', c.throttle, 0.0, 1.0),
            ('Steer:', c.steer, -1.0, 1.0),
            ('Brake:', c.brake, 0.0, 1.0),
            ('Reverse:', c.reverse),
            ('Hand brake:', c.hand_brake),
            ('Manual:', c.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        self._info_text += [
            '', 'Collision:', collision, '', 'Number of vehicles: % 8d' % (len(vehicles)-1)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: np.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != self.vehicle.id]
            for d, vehicle in sorted(vehicles):
                if d > 800.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))
        self.render(self.display)
        
    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.notification('Weather: %s' % preset[1])
        self.world.set_weather(preset[0])

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        display.fill((255,255,255))

        size = self._surface.get_size()
        desired_aspect_ratio = float(size[0]) / float(size[1])
        dim1 = (self.dim[0], round(self.dim[0]/desired_aspect_ratio))
        dim2 = (round(self.dim[1]*desired_aspect_ratio), self.dim[1])
        scaled_size = min(dim1, dim2)
        display.blit(pygame.transform.scale(self._surface, scaled_size), (0, 0))

        info_surface = pygame.Surface((220, self.dim[1]))
        alpha = 100 if self._show_info else 0
        info_surface.set_alpha(alpha)
        display.blit(info_surface, (0, 0))
        v_offset = 4
        bar_h_offset = 100
        bar_width = 106
        for item in self._info_text:
            if v_offset + 18 > self.dim[1]:
                break
            if isinstance(item, list):
                if len(item) > 1:
                    points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                    pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                item = None
                v_offset += 18
            elif isinstance(item, tuple):
                if isinstance(item[1], bool):
                    rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                    pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                else:
                    rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                    pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                    f = (item[1] - item[2]) / (item[3] - item[2])
                    if item[2] < 0.0:
                        rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                    else:
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                    pygame.draw.rect(display, (255, 255, 255), rect)
                item = item[0]
            if item:  # At this point has to be a str.
                surface = self._font_mono.render(item, True, (255, 255, 255))
                display.blit(surface, (8, v_offset))
            v_offset += 18

        self._notifications.render(display)



class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)





# ==============================================================================
# -- aggregation ---------------------------------------------------------------
# ==============================================================================


class PyGameInteraction(object):
    def __init__(self, client, vehicle, sensors_master, config):
        '''
            Args:
            config: need to contain:
                config.width
                config.height
                config.use_kb_control
        '''
        width, height = config.get('width', 1000), config.get('height', 600)
        self.use_kb_control = config.get('use_kb_control', True)
        
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.clock = pygame.time.Clock()
        self.client = client
        self.hud = HUD(client, client.get_world(), vehicle, sensors_master, self.display)
        self.kb_control = KeyboardControl(self.hud, self.display)


    def tick(self):
        self.clock.tick_busy_loop(0)
        if self.use_kb_control: self.kb_control.parse_events(self.display, self.clock)
        self.hud.tick(self.clock)
        pygame.display.flip()


    def destroy(self):
        pygame.quit()
