/*
 * Kilobots doing simulated collective decision-making with Bayesian models
 *
 * This initial version is for SINGLE FEATURE (color)
 *
 * Created 2018-12 by Julia Ebert
 */

#include <kilosim/Kilobot.h>
#include "incbeta.h"

// Pseudo-booleans for convenience
#define FALSE 0
#define TRUE 1

namespace Kilosim
{

typedef struct neighbor_info_array_t
{
    // One entry (row) in a table of observations from neighbors
    float measured_distance;
    uint16_t id;
    uint16_t obs_ind;
    uint32_t time_first_heard_from;
} neighbor_info_array_t;

class BayesBot : public Kilobot
{
public:
    // Variables for aggregators
    uint32_t dark_count = 0;      // beta in Beta distribution
    uint32_t light_count = 0;     // alpha in Beta distribution
    int8_t decision = -1;         // 0 or 1 value of decision, once made
    uint16_t observation_ind = 0; // Index observations so receivers know if it's new

    // Public allows prior to be set from main function in initialization
    // Setting these different from 1,1 changes from uniform prior
    uint32_t light_prior = 1; // alpha prior
    uint32_t dark_prior = 1;  // beta prior
    uint8_t use_positive_feedback = TRUE;
    double credible_thresh = 0.95; // % of prob. that must be above/below 0.5
    uint8_t allow_simultaneity = TRUE;
    uint32_t observe_step_time; // Time between observations (seconds)
    uint32_t disseminate_dur;   // in kiloticks (only relevant if !allow_simultaneity)

private:
    // Easier-to-read color values
    const uint8_t DARK = 0;
    const uint8_t GRAY = 1;
    const uint8_t LIGHT = 2;
    uint8_t curr_light_level; // DARK, GRAY, or LIGHT, initialized in setup()

    // Task states
    const uint8_t OBSERVE = 0;
    const uint8_t DISSEMINATE = 1;
    const uint8_t OBSERVE_DISSEMINATE = 2; // Both at once
    uint8_t state;
    uint32_t state_change_timer;

    // Random walk patterns
    const uint8_t RW_INIT = 0;
    const uint8_t RW_STRAIGHT = 1;
    const uint8_t RW_TURN = 2;
    uint8_t rw_state = RW_INIT;
    const uint8_t TURN_LEFT = 0;
    const uint8_t TURN_RIGHT = 1;
    uint32_t rw_last_changed = 0;                 // kilotick when rw_state last changed
    uint32_t rw_mean_straight_dur = 240 * SECOND; // kiloticks
    uint32_t rw_max_turn_dur = 12 * SECOND;       // kiloticks
    // Actual turn/straight durations are set at beginning of transition to that state
    uint32_t rw_state_dur;
    uint8_t is_feature_detect_safe = FALSE; // Feature detection needs to be enabled in loop

    // Bounce out of gray area when it gets there (like a screensaver)
    const uint8_t BOUNCE = 100;
    uint8_t bounce_turn_state;

    // Observation variables/parameters
    uint32_t last_observation_tick;
    uint8_t observation; // 0 or 1
    uint8_t new_observation = FALSE;

    // Messages/communication
#define NEIGHBOR_INFO_ARRAY_SIZE 100
    neighbor_info_array_t neighbor_info_array[NEIGHBOR_INFO_ARRAY_SIZE];
    message_t rx_message_buffer;
    distance_measurement_t rx_distance_buffer;
    uint8_t new_message = 0;
    uint8_t neighbor_info_array_locked = FALSE;
    uint32_t neighbor_info_array_timeout = 900 * SECOND; // kiloticks
    message_t tx_message_data;

    // DEBUG values
    double beta_thresh_val = 0.5;

    //--------------------------------------------------------------------------
    // GENERALLY USEFUL FUNCTIONS
    //--------------------------------------------------------------------------

    uint32_t uniform_rand(uint32_t max_val)
    {
        // Generate a random int from 0 to max_val
        uint32_t rand_val = (uint32_t)rand_hard() / 255.0 * max_val;
        return rand_val;
    }

    uint32_t exp_rand(double mean_val)
    {
        // Generate random value from exponential distribution with mean mean_val
        // According to: http://stackoverflow.com/a/11491526/2552873
        // Generate random float (0,1)
        double unif_val = (double)rand_hard() / 255.0;
        uint32_t exp_val = uint32_t(-log(unif_val) * mean_val);
        return exp_val;
    }

    uint16_t count_neighbors()
    {
        // Count how many neighbors in neighbor info array (how many with non-zero ID)
        uint16_t num_neighbors = 0;
        for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; i++)
        {
            if (neighbor_info_array[i].id != 0)
                num_neighbors++;
        }
        return num_neighbors;
    }

    uint8_t find_wall_collision()
    {
        // Use light sensor to detect if outside the black/white area (into gray)
        uint8_t light_level = detect_light_level();
        if (light_level == GRAY)
            return 1;
        else
            return 0;
    }

    void print_neighbor_info_array()
    {
        printf("\n\n\nOwn ID = %d\tObservation = %u\tBeta = (%u, %u)\tDecision = %u\n",
               id, observation,
               light_count, dark_count,
               decision);

        printf("Index\tID\tFeature\tBelief\tD_meas.\tN_Heard\tTime\n\r");
        for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i)
        {
            if (neighbor_info_array[i].id != 0)
            {
                printf("%u\t%d\t%u\t%u\n\r",
                       //  1   2   3   4   5   6   7
                       i,
                       neighbor_info_array[i].id,
                       ((uint8_t)neighbor_info_array[i].measured_distance),
                       (uint16_t)(kilo_ticks - neighbor_info_array[i].time_first_heard_from));
            }
        }
    }

    void initialize_neighbor_info_array()
    {
        for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i)
        {
            neighbor_info_array[i].id = 0;
        }
    }

    void prune_neighbor_info_array()
    {
        // Get rid of neighbors from timeout table after a fixed length of time
        for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i)
        {
            if (kilo_ticks > (neighbor_info_array[i].time_first_heard_from + neighbor_info_array_timeout))
            {
                neighbor_info_array[i].id = 0;
            }
        }
    }

    void update_neighbor_info_array(message_t *m, distance_measurement_t *d)
    {
        /*
         * Add an incoming message to the array of received messages info.
         * If a neighbor is not in the table, add it. Old data will be removed
         * after a fixed-length time out. New data from the robot will be used
         * only if it is after the timeout (aka not in the table)
         */
        bool can_insert = false;
        bool new_entry;
        uint8_t index_to_insert;

        uint16_t rx_id = (((uint16_t)m->data[0]) << 8) | ((uint16_t)(m->data[1]));
        uint8_t obs_val = m->data[2]; // observation value. (0=dark, 1=light)
        uint16_t rx_obs_ind = (((uint16_t)m->data[3]) << 8) | ((uint16_t)(m->data[4]));

        // printf("%u:\t%u\t%u\t%u\n", id, rx_id, obs_val, rx_obs_ind);

        for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i)
        {
            if (neighbor_info_array[i].id == rx_id)
            {
                // Message from the neighbor is already in table
                can_insert = true;
                new_entry = false;
                index_to_insert = i;
            }
        }
        if (!can_insert)
        {
            for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i)
            {
                if (neighbor_info_array[i].id == 0)
                {
                    // There's an empty spot in the table
                    can_insert = true;
                    new_entry = true;
                    index_to_insert = i;
                    break;
                }
            }
        }
        if (!can_insert)
        {
            // This robot isn't in the array, and there isn't an empty space
            // So kick out the oldest message/robot
            uint8_t earliest_heard_time = 0;
            uint8_t earliest_heard_time_index;
            for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i)
            {
                if (neighbor_info_array[i].time_first_heard_from > earliest_heard_time)
                {
                    earliest_heard_time = neighbor_info_array[i].time_first_heard_from;
                    earliest_heard_time_index = i;
                }
            }
            // Replace oldest entry if table is full
            can_insert = true;
            new_entry = true;
            index_to_insert = earliest_heard_time_index;
        }

        if (can_insert && new_entry)
        {
            neighbor_info_array[index_to_insert].time_first_heard_from = kilo_ticks;
            neighbor_info_array[index_to_insert].id = rx_id;
            // Update Beta model with incoming observations ONLY if observation index changed
            if (neighbor_info_array[index_to_insert].obs_ind != rx_obs_ind)
                update_beta(obs_val);
            neighbor_info_array[index_to_insert].obs_ind = rx_obs_ind;
        }
    }

    //--------------------------------------------------------------------------
    // AUXILIARY FUNCTIONS
    //--------------------------------------------------------------------------

    void random_walk(uint32_t mean_straight_dur, uint32_t max_turn_dur)
    {
        // Non-blocking random walk, iterating between turning and walking states
        // Durations are in kiloticks

        uint8_t wall_hit = find_wall_collision();
        if (wall_hit != 0 && rw_state != BOUNCE)
        {
            // Check for wall collision before anything else
            rw_state = BOUNCE;
            is_feature_detect_safe = FALSE;
            bounce_init(wall_hit);
        }
        else if (rw_state == BOUNCE && curr_light_level != GRAY)
        {
            // end bounce phase
            rw_state = RW_INIT;
        }
        else if (rw_state == RW_INIT)
        {
            // Set up variables
            rw_state = RW_STRAIGHT;
            is_feature_detect_safe = TRUE;
            rw_last_changed = kilo_ticks;
            rw_state_dur = exp_rand(mean_straight_dur);
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
        else if (rw_state == RW_STRAIGHT && kilo_ticks > rw_last_changed + rw_state_dur)
        {
            // Change to turn state
            rw_last_changed = kilo_ticks;
            rw_state = RW_TURN;
            is_feature_detect_safe = FALSE;
            // Select turning duration in kilo_ticks
            rw_state_dur = uniform_rand(max_turn_dur);
            // Set turning direction
            spinup_motors();
            bool is_turn_left = rand_hard() & 1;
            if (is_turn_left)
            {
                set_motors(kilo_turn_left, 0);
            }
            else
            { // turn right
                set_motors(0, kilo_turn_left);
            }
        }
        else if (rw_state == RW_TURN && kilo_ticks > rw_last_changed + rw_state_dur)
        {
            // Change to straight state
            rw_last_changed = kilo_ticks;
            rw_state = RW_STRAIGHT;
            is_feature_detect_safe = TRUE;
            // Select staight movement duration
            rw_state_dur = exp_rand(mean_straight_dur);
            // Set turning direction
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
    }

    void bounce_init(uint8_t wall_hit)
    {
        // Start the bounce movement
        // Also set rw_state/ef_state to BOUNCE in the calling function
        // Don't forget to end the bounce phase in the calling function as well...

        // Now it doesn't know what wall it hit. Randomly pick a direction to turn?
        if (rand() % 2 == 0)
            bounce_turn_state = TURN_LEFT;
        else
            bounce_turn_state = TURN_RIGHT;
        // Start bounce
        spinup_motors();
        if (bounce_turn_state == TURN_LEFT)
            set_motors(kilo_turn_left, 0);
        else
            set_motors(0, kilo_turn_right);
    }

    uint8_t detect_light_level()
    {
        // Detect/return light level (DARK = [0,250), GRAY = [250-750), LIGHT = [750-1024])
        // This version is for MONOCHROME FEATURES, where all light is assumed to be in channel 0 (red)
        // Get current light level
        uint16_t light = get_ambientlight();
        if (light < 250)
            return DARK;

        else if (light < 750)
            return GRAY;
        else
            return LIGHT;
    }

    double update_decision()
    {
        /* Check whether a decision can be made.
         * Uses the credible interval for the Beta distribution
         * 0 = decide low/dark
         * 1 = decide high/light
         * -1 = undecided
         */
        // % of probability mass below 0.5
        double beta_thresh = incbeta(light_count + light_prior, dark_count + dark_prior, 0.5);
        // std::cout << "[" << id << "]\t" << light_count + light_prior << ", " << dark_count + dark_prior << "\t" << beta_thresh << std::endl;
        if (beta_thresh > credible_thresh)
            decision = 0;
        else if (beta_thresh < (1 - credible_thresh))
            decision = 1;
        else
            decision = -1;
        return beta_thresh;
    }

    void update_beta(uint8_t obs)
    {
        // Add a 0/1 to the Beta distribution counts
        dark_count += 1 - obs;
        light_count += obs;
    }

    void observe_color()
    {
        /*
         * Make an observation of the color after every fixed-length step
         * This sets the `observation` value and `new_observation` flag
         */
        if (last_observation_tick + observe_step_time * SECOND <= kilo_ticks)
        {
            if (curr_light_level != GRAY)
            {
                new_observation = TRUE;
                if (curr_light_level == DARK)
                    observation = 0;
                else
                    observation = 1;
            }
            // Even if in gray, wait a round to re-observe
            last_observation_tick = kilo_ticks;
        }
    }

    //--------------------------------------------------------------------------
    // REQUIRED KILOBOT FUNCTIONS
    //--------------------------------------------------------------------------

    void setup()
    {
        // Deal with feature/bug of limited battery life
        // battery = 60 * 60 * SECOND * 20; // 20 hours (in kiloticks)
        battery = 100000 * SECOND;

        curr_light_level = detect_light_level();
        rw_last_changed = kilo_ticks;
        set_color(RGB(0.5, 0.5, 0.5));
        initialize_neighbor_info_array();
        if (allow_simultaneity)
            state = OBSERVE_DISSEMINATE;
        else
            state = OBSERVE;
    }

    void loop()
    {
        // DEBUG
        //std::cout << id << ":  " << x << ", " << y << std::endl;

        curr_light_level = detect_light_level();
        // Movement depending on state/feature
        random_walk(rw_mean_straight_dur, rw_max_turn_dur);

        if (state == OBSERVE || state == OBSERVE_DISSEMINATE)
        {
            // Observe
            observe_color();
            if (new_observation)
            {
                update_beta(observation);
                if (decision == -1)
                    beta_thresh_val = update_decision();
                new_observation = FALSE;
                observation_ind++;
                if (!allow_simultaneity)
                {
                    // Change to disseminating new observation
                    state = DISSEMINATE;
                    state_change_timer = kilo_ticks;
                }
            }
        }

        // Process new received message
        // Update beta distribution if it's a new observation (by index)
        neighbor_info_array_locked = TRUE;
        if (new_message)
        {
            // (This also runs update_beta)
            update_neighbor_info_array(&rx_message_buffer, &rx_distance_buffer);
            new_message = FALSE;
            if (decision == -1)
                beta_thresh_val = update_decision();
        }
        prune_neighbor_info_array();
        neighbor_info_array_locked = FALSE;

        // Check for and update decisions (only if undecided)
        if (decision == 0)
            set_color(RGB(1, 0, 0));
        else if (decision == 1)
            set_color(RGB(0, 1, 0));
        else
        {
            set_color(RGB(beta_thresh_val * .8, (1 - beta_thresh_val) * .8, 0.5 * .8));
            // set_color(RGB(0, 0, 1));
            // printf("%u:\t%u\t%u\n", id, light_count, dark_count);
        }

        // Switch back to observation (if can't do everything simultaneously)
        // and past disseminate_dur
        if (!allow_simultaneity && state == DISSEMINATE &&
            state_change_timer + disseminate_dur <= kilo_ticks)
        {
            state == OBSERVE;
        }
    }

    void update_tx_message_data()
    {
        tx_message_data.type = NORMAL;
        // ID
        tx_message_data.data[0] = ((uint8_t)((id & 0xff00) >> 8));
        tx_message_data.data[1] = ((uint8_t)(id & 0x00ff));
        // Observation value (0=dark, 1=light) OR decision
        if (decision != -1 && use_positive_feedback)
            tx_message_data.data[2] = decision;
        else
            tx_message_data.data[2] = observation;
        // Include sender's personal observation indexing (so receiver knows if new observation)
        tx_message_data.data[3] = ((uint8_t)((observation_ind & 0xff00) >> 8));
        tx_message_data.data[4] = ((uint8_t)(observation_ind & 0x00ff));
        tx_message_data.crc = message_crc(&tx_message_data);
    }

    void message_rx(message_t *msg, distance_measurement_t *dist)
    {
        if (!neighbor_info_array_locked)
        {
            rx_message_buffer = (*msg);
            rx_distance_buffer = (*dist);
            new_message = TRUE;
            // TODO: Needs to be moved out of here (to loop function) for actual kilobots
        }
    }

    message_t *message_tx()
    {
        if (state == DISSEMINATE || state == OBSERVE_DISSEMINATE)
        {
            update_tx_message_data();
            return &tx_message_data;
        }
        else
        {
            return NULL;
        }
    }

    void message_tx_success() {}
};
} // namespace Kilosim
