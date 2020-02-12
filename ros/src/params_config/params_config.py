import yaml
import rospy

class ParamsConfig:
    __instance = None
    __config = None
    __config_present = True

    @staticmethod 
    def getInstance():
        """ Static access method. """
        if ParamsConfig.__instance == None:
            ParamsConfig()
        return ParamsConfig.__instance
    def __init__(self):
        """ Virtually private constructor. """
        if ParamsConfig.__instance != None:
            raise Exception("This class is a Singleton!")
        else:
            ParamsConfig.__instance = self

    def getConfig(self):
        if ParamsConfig.__config_present == False:
            return None

        if ParamsConfig.__config is None:
            try:
                config_string = rospy.get_param("/params_config")
                ParamsConfig.__config = yaml.load(config_string)
            except KeyError:
                rospy.loginfo("Custom parameters not used")
                ParamsConfig.__config_present = False
                ParamsConfig.__config = None


        return ParamsConfig.__config
