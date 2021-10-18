class Motor{
    private:
        int c_speed ;
        int c_directionAMotor;
        int c_directionBMotor;
        int c_pwmMotor ;
        int c_duration ;
    public:
        Motor(int directionAMotor, int directionBMoteur, int pwmMotor);
        void init();
        void run(int speed);
        void stop();
};
