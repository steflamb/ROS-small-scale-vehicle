import struct

class ViconTrackerUtils:
    def __init__(self):
        self.FRAME_NUMBER_OFFSET = 0
        self.FRAME_NUMBER_SIZE = 4
        self.ITEMS_IN_BLOCK_SIZE = 1
        self.OBJECT_ID_SIZE = 1
        self.ITEM_DATA_SIZE_SIZE = 2
        self.OBJECT_NAME_SIZE = 24
        self.TRANSLATION_SIZE = 8
        self.ROTATION_SIZE = 8
        self.OBJECT_ID_OFFSET = self.FRAME_NUMBER_OFFSET + self.FRAME_NUMBER_SIZE + self.ITEMS_IN_BLOCK_SIZE
        self.ITEM_DATA_SIZE_OFFSET = self.OBJECT_ID_OFFSET + self.OBJECT_ID_SIZE
        self.OBJECT_NAME_OFFSET = self.ITEM_DATA_SIZE_OFFSET + self.ITEM_DATA_SIZE_SIZE
        self.TRANSLATION_X_OFFSET = 32
        self.TRANSLATION_Y_OFFSET = 40
        self.TRANSLATION_Z_OFFSET = 48
        self.ROTATION_X_OFFSET = 56
        self.ROTATION_Y_OFFSET = 64
        self.ROTATION_Z_OFFSET = 72

    def parse_translation(self, data):
        translation_x = struct.unpack("<d", data[self.TRANSLATION_X_OFFSET:self.TRANSLATION_X_OFFSET + 8])[0]
        translation_y = struct.unpack("<d", data[self.TRANSLATION_Y_OFFSET:self.TRANSLATION_Y_OFFSET + 8])[0]
        translation_z = struct.unpack("<d", data[self.TRANSLATION_Z_OFFSET:self.TRANSLATION_Z_OFFSET + 8])[0]
        return translation_x, translation_y, translation_z

    def parse_rotation(self, data):
        rotation_x_rad = struct.unpack("<d", data[self.ROTATION_X_OFFSET:self.ROTATION_X_OFFSET + 8])[0]
        rotation_y_rad = struct.unpack("<d", data[self.ROTATION_Y_OFFSET:self.ROTATION_Y_OFFSET + 8])[0]
        rotation_z_rad = struct.unpack("<d", data[self.ROTATION_Z_OFFSET:self.ROTATION_Z_OFFSET + 8])[0]
        return rotation_x_rad, rotation_y_rad, rotation_z_rad

    def parse_frame_number(self, data):
        frame_number = struct.unpack("<I", data[self.FRAME_NUMBER_OFFSET:self.FRAME_NUMBER_OFFSET + self.FRAME_NUMBER_SIZE])[0]
        return frame_number

    def parse_object_id(self, data):
        object_id = struct.unpack("<B", data[self.OBJECT_ID_OFFSET:self.OBJECT_ID_OFFSET + self.OBJECT_ID_SIZE])[0]
        return object_id

    def parse_object_name(self, data):
        object_name = data[self.OBJECT_NAME_OFFSET:self.OBJECT_NAME_OFFSET + self.OBJECT_NAME_SIZE].decode().rstrip('\x00')
        # Replace invalid characters and convert to lowercase
        #object_name = re.sub(r'[^a-zA-Z0-9_]', '_', object_name).lower()
        object_name = re.sub(r'[^a-zA-Z0-9_]', '_', object_name)
        return object_name