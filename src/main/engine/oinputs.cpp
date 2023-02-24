/***************************************************************************
    Process Inputs.
    
    - Read & Process inputs and controls.
    - Note, this class does not contain platform specific code.
    
    Copyright Chris White.
    See license.txt for more details.
***************************************************************************/

#include <iostream>

#include "engine/ocrash.hpp"
#include "engine/oinputs.hpp"
#include "engine/ostats.hpp"

OInputs oinputs;

OInputs::OInputs(void)
{
}

OInputs::~OInputs(void)
{
}

void OInputs::init()
{
    input_steering  = STEERING_CENTRE;
    steering_old    = STEERING_CENTRE;
    steering_adjust = 0;
    acc_adjust      = 0;
    brake_adjust    = 0;
    steering_change = 0;

    steering_inc = config.controls.steer_speed;
    acc_inc      = config.controls.pedal_speed * 4;
    brake_inc    = config.controls.pedal_speed * 4;

    input_acc   = 0;
    input_brake = 0;
    gear        = false;
    crash_input = 0;
    delay1      = 0;
    delay2      = 0;
    delay3      = 0;
    coin1       = false;
    coin2       = false;
}

void OInputs::tick()
{
#if 1
    touch_object_control();
#else
    // Digital Controls: Simulate Analog
    if (!input.analog || !input.gamepad)
    {
        digital_steering();
        digital_pedals();
    }
    // Analog Controls
    else
    {
        input_steering = input.a_wheel;

        // Analog Pedals
        if (input.analog == 1)
        {
            input_acc      = input.a_accel;
            input_brake    = input.a_brake;
        }
        // Digital Pedals
        else
        {
            digital_pedals();
        }
    }
#endif
}
//获得det[i][j]余子式行列式
vector<vector<double> > getComplementMinor(vector<vector<double> > det,int i,int j)
{

	int n=det.size(),m=det[0].size();//n为det的行，m为det的列；
	vector<vector<double> > ans(n-1);//保存获得的结果
	for(int k=0;k<n-1;k++)
	for(int l=0;l<n-1;l++)
	{
		ans[k].push_back(det[k<i?k:k+1][l<j?l:l+1]);
	}
	return ans;
}

double getDetVal(vector<vector<double> > det)
{
    double ans=0;
	int n=det.size(),m=det[0].size();//n为det的行，m为det的列；
	if(n != m)
    {
    	 cout<<" 您输入的矩阵不是方阵！求么子行列式！";
    	 exit(1);
	}
	if(det.size() == 1)
	return det[0][0];

	for(int i=0;i<m;i++)
	{
		ans+=det[0][i] * pow(-1,i)*getDetVal(getComplementMinor(det,0,i));
	}
	return ans;
}

double _line_length(pair<double, double> a, pair<double, double> b)
{
    return sqrt(pow(b.first - a.first, 2) + pow(b.second - a.second, 2));
}

#define PI 3.14159265

double _vector_angle(pair<double, double> a, pair<double, double> b)
{
    return acos((a.first * b.first + a.second*b.second) /
    (sqrt(pow(a.first, 2) + pow(a.second, 2)) * sqrt(pow(b.first, 2) + pow(b.second, 2))))
    * 180.0 / PI;
}

inline pair<double, double> operator-(pair<double, double>& a, pair<double, double>& b)
{
    return make_pair(b.first - a.first, b.second - a.second);
}

void OInputs::touch_object_control()
{
    pair<double, double> p[3];
    size_t index = 0;
#if 1
    for(auto& touch : input.touch_points)
    {
        if(touch.pressure > 0.0)
        {
            p[index].first = touch.x * 1920;
            p[index].second = touch.y * 1080;
            index++;
        }

        if(index >= std::size(p))
        {
            break;
        }
    }
#else
    pair<double, double> touch[] = {{0.8, 0.8}, {0.75, 0.75}, {0.85, 0.75}};
    for(auto& t : touch)
    {
        p[index].first = t.first * 1920;
        p[index].second = t.second * 1080;
        index++;

        if(index >= std::size(p))
        {
            break;
        }
    }
#endif

    if(index == std::size(p))
    {
        pair<double, double> circle_center;
        double triangle_side_length[3];
        double angle = 0;

        circle_center.first = getDetVal({
            {pow(p[0].first,2) + pow(p[0].second, 2), p[0].second, 1},
            {pow(p[1].first,2) + pow(p[1].second, 2), p[1].second, 1},
            {pow(p[2].first,2) + pow(p[2].second, 2), p[2].second, 1},
        }) / 2 / getDetVal({
            {p[0].first, p[0].second, 1},
            {p[1].first, p[1].second, 1},
            {p[2].first, p[2].second, 1},
        });

        circle_center.second = getDetVal({
            {p[0].first, pow(p[0].first,2) + pow(p[0].second, 2), 1},
            {p[1].first, pow(p[1].first,2) + pow(p[1].second, 2), 1},
            {p[2].first, pow(p[2].first,2) + pow(p[2].second, 2), 1},
        }) / 2 / getDetVal({
            {p[0].first, p[0].second, 1},
            {p[1].first, p[1].second, 1},
            {p[2].first, p[2].second, 1},
        });

        triangle_side_length[0] = _line_length(p[1], p[2]);
        triangle_side_length[1] = _line_length(p[0], p[2]);
        triangle_side_length[2] = _line_length(p[1], p[0]);
        if(abs(triangle_side_length[0] - triangle_side_length[1]) < 0.001)
        {
            angle = _vector_angle(p[2] - circle_center, make_pair(1, 0));
        }
        else if(abs(triangle_side_length[0] - triangle_side_length[2]) < 0.001)
        {
            angle = _vector_angle(p[1] - circle_center, make_pair(1, 0));
        }
        else
        {
            angle = _vector_angle(p[0] - circle_center, make_pair(1, 0));
        }

        if (circle_center.second > 1080 / 3)
        {
            input_acc += acc_inc;
            if (input_acc > 0xFF) input_acc = 0xFF;

            input_brake -= brake_inc;
            if (input_brake < 0) input_brake = 0;
        }
        else
        {
            input_acc -= acc_inc;
            if (input_acc < 0) input_acc = 0;

            input_brake += brake_inc;
            if (input_brake > 0xFF) input_brake = 0xFF;
        }

        input_steering = angle / 180 * (STEERING_MAX - STEERING_MIN) + STEERING_MIN;
        if (input_steering < STEERING_MIN)
        {
            input_steering = STEERING_MIN;
        }
        else if (input_steering > STEERING_MAX)
        {
            input_steering = STEERING_MAX;
        }
    }
}

// DIGITAL CONTROLS: Digital Simulation of analog steering
void OInputs::digital_steering()
{
    // ------------------------------------------------------------------------
    // STEERING
    // ------------------------------------------------------------------------
    if (input.is_pressed(Input::LEFT))
    {
        // Recentre wheel immediately if facing other way
        if (input_steering > STEERING_CENTRE) input_steering = STEERING_CENTRE;

        input_steering -= steering_inc;
        if (input_steering < STEERING_MIN) input_steering = STEERING_MIN;
    }
    else if (input.is_pressed(Input::RIGHT))
    {
        // Recentre wheel immediately if facing other way
        if (input_steering < STEERING_CENTRE) input_steering = STEERING_CENTRE;

        input_steering += steering_inc;
        if (input_steering > STEERING_MAX) input_steering = STEERING_MAX;
    }
    // Return steering to centre if nothing pressed
    else
    {
        if (input_steering < STEERING_CENTRE)
        {
            input_steering += steering_inc;
            if (input_steering > STEERING_CENTRE)
                input_steering = STEERING_CENTRE;
        }
        else if (input_steering > STEERING_CENTRE)
        {
            input_steering -= steering_inc;
            if (input_steering < STEERING_CENTRE)
                input_steering = STEERING_CENTRE;
        }
    }
}

// DIGITAL CONTROLS: Digital Simulation of analog pedals
void OInputs::digital_pedals()
{
    // ------------------------------------------------------------------------
    // ACCELERATION
    // ------------------------------------------------------------------------

    if (input.is_pressed(Input::ACCEL))
    {
        input_acc += acc_inc;
        if (input_acc > 0xFF) input_acc = 0xFF;
    }
    else
    {
        input_acc -= acc_inc;
        if (input_acc < 0) input_acc = 0;
    }

    // ------------------------------------------------------------------------
    // BRAKE
    // ------------------------------------------------------------------------

    if (input.is_pressed(Input::BRAKE))
    {
        input_brake += brake_inc;
        if (input_brake > 0xFF) input_brake = 0xFF;
    }
    else
    {
        input_brake -= brake_inc;
        if (input_brake < 0) input_brake = 0;
    }
}

void OInputs::do_gear()
{
    // ------------------------------------------------------------------------
    // GEAR SHIFT
    // ------------------------------------------------------------------------

    // Automatic Gears: Don't do anything
    if (config.controls.gear == config.controls.GEAR_AUTO)
        return;

    else
    {
        // Manual: Cabinet Shifter
        if (config.controls.gear == config.controls.GEAR_PRESS)
            gear = !input.is_pressed(Input::GEAR1);

        // Manual: Two Separate Buttons for gears
        else if (config.controls.gear == config.controls.GEAR_SEPARATE)
        {
            if (input.has_pressed(Input::GEAR1))
                gear = false;
            else if (input.has_pressed(Input::GEAR2))
                gear = true;
        }

        // Manual: Keyboard/Digital Button
        else
        {
            if (input.has_pressed(Input::GEAR1))
                gear = !gear;
        }
    }
}

// Adjust Analogue Inputs
//
// Read, Adjust & Write Analogue Inputs
// In the original, these values are set during a H-Blank routine
//
// Source: 74E2

void OInputs::adjust_inputs()
{
    // Cap Steering Value
    if (input_steering < STEERING_MIN) input_steering = STEERING_MIN;
    else if (input_steering > STEERING_MAX) input_steering = STEERING_MAX;

    if (crash_input)
    {
        crash_input--;
        int16_t d0 = ((input_steering - 0x80) * 0x100) / 0x70;
        if (d0 > 0x7F) d0 = 0x7F;
        else if (d0 < -0x7F) d0 = -0x7F;
        steering_adjust = ocrash.crash_counter ? 0 : d0;
    }
    else
    {
        // no_crash1:
        int16_t d0 = input_steering - steering_old;
        steering_old = input_steering;
        steering_change += d0;
        d0 = steering_change < 0 ? -steering_change : steering_change;

        // Note the below line "if (d0 > 2)" causes a bug in the original game
        // whereby if you hold the wheel to the left whilst stationary, then accelerate the car will veer left even
        // when the wheel has been centered
        if (config.engine.fix_bugs || d0 > 2)
        {
            steering_change = 0;
            // Convert input steering value to internal value
            d0 = ((input_steering - 0x80) * 0x100) / 0x70;
            if (d0 > 0x7F) d0 = 0x7F;
            else if (d0 < -0x7F) d0 = -0x7F;
            steering_adjust = ocrash.crash_counter ? 0 : d0;
        }
    }

    // Cap & Adjust Acceleration Value
    int16_t acc = input_acc;
    if (acc > PEDAL_MAX) acc = PEDAL_MAX;
    else if (acc < PEDAL_MIN) acc = PEDAL_MIN;
    acc_adjust = ((acc - 0x30) * 0x100) / 0x61;

    // Cap & Adjust Brake Value
    int16_t brake = input_brake;
    if (brake > PEDAL_MAX) brake = PEDAL_MAX;
    else if (brake < PEDAL_MIN) brake = PEDAL_MIN;
    brake_adjust = ((brake - 0x30) * 0x100) / 0x61;
}

// Simplified version of do credits routine. 
// I have not ported the coin chute handling code, or dip switch routines.
//
// Returns: 0 (No Coin Inserted)
//          1 (Coin Chute 1 Used)
//          2 (Coin Chute 2 Used)
//          3 (Key Pressed / Service Button)
//
// Source: 0x6DE0
uint8_t OInputs::do_credits()
{
    if (input.has_pressed(Input::COIN))
    {
        if (!config.engine.freeplay && ostats.credits < 9)
        {
            ostats.credits++;
            // todo: Increment credits total for bookkeeping
            osoundint.queue_sound(sound::COIN_IN);
        }
        return 3;
    }
    else if (coin1)
    {
        coin1 = false;
        if (!config.engine.freeplay && ostats.credits < 9)
        {
            ostats.credits++;
            osoundint.queue_sound(sound::COIN_IN);
        }
        return 1;
    }
    else if (coin2)
    {
        coin2 = false;
        if (!config.engine.freeplay && ostats.credits < 9)
        {
            ostats.credits++;
            osoundint.queue_sound(sound::COIN_IN);
        }
        return 2;
    }
    return 0;
}

// ------------------------------------------------------------------------------------------------
// Menu Selection Controls
// ------------------------------------------------------------------------------------------------

bool OInputs::is_analog_l()
{
    if (input_steering < STEERING_CENTRE - 0x10)
    {
        if (--delay1 < 0)
        {
            delay1 = DELAY_RESET;
            return true;
        }
    }
    else
        delay1 = DELAY_RESET;
    return false;
}

bool OInputs::is_analog_r()
{
    if (input_steering > STEERING_CENTRE + 0x10)
    {
        if (--delay2 < 0)
        {
            delay2 = DELAY_RESET;
            return true;
        }
    }
    else
        delay2 = DELAY_RESET;
    return false;
}

bool OInputs::is_analog_select()
{
    if (input_acc > 0x90)
    {
        if (--delay3 < 0)
        {
            delay3 = DELAY_RESET;
            return true;
        }
    }
    else
        delay3 = DELAY_RESET;
    return false;
}