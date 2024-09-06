use rtt_target::rprintln;

pub struct DateTime {
    pub year: u32,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl DateTime {
    pub fn new(year: u32, month: u8, day: u8, hour: u8, minute: u8, second: u8) -> Self {
        DateTime { year, month, day, hour, minute, second }
    }

    pub fn update(&mut self, mut year: u32, mut month: u8, mut day: u8, mut hour: u8, mut minute: u8, mut second: u8) {
        // Calculate the total number of seconds to add
        //rprintln!("year {:?}", year);
        let seconds_to_add = year as u64 * 60 * 60 * 24 * 365 + month as u64 * 60 * 60 * 24 * 30 + day as u64 * 60 * 60 * 24 + hour as u64 * 60 * 60 + minute as u64 * 60 + second as u64;
        //rprintln!("seconds_to_add {:?}", seconds_to_add);
        let mut total_seconds = DateTime::convert_datetime_to_seconds(&DateTime::new(self.year, self.month, self.day, self.hour, self.minute, self.second)) + seconds_to_add;
        //rprintln!("total_seconds {:?}", total_seconds);
        
        *self = DateTime::convert_seconds_to_datetime(total_seconds)
    }
    
    pub fn set_time(&mut self, hour: u8, minute: u8, second: u8) {
        // Validate input values
        if hour < 24 && minute < 60 && second < 60 {
            // Only update if all values are valid
            self.hour = hour;
            self.minute = minute;
            self.second = second;
        } else {
            rprintln!("Invalid time provided.");
        }
    }

    pub fn set_date(&mut self, year: u32, month: u8, day: u8) {
        // Validate input values
        if month > 0 && month <= 12 && day > 0 && day <= DateTime::get_days_in_month(month, year) {
            // Check for February in leap year
            if month == 2 && day == 29 && !DateTime::is_leap_year(year) {
                rprintln!("Invalid date provided.");
            } else {
                // Only update if all values are valid
                self.year = year;
                self.month = month;
                self.day = day;
            }
        } else {
            rprintln!("Invalid date provided.");
        }
    }
    

    pub fn get_time(&self) -> (u8, u8, u8) {
        (self.hour, self.minute, self.second)
    }

    pub fn get_date(&self) -> (u32, u8, u8) {
        (self.year, self.month, self.day)
    }

    fn is_leap_year(year: u32) -> bool {
        (year % 4 == 0 && year % 100 != 0) || year % 400 == 0
    }

    fn get_days_in_months(year: u32) -> [u8; 12] {
        if DateTime::is_leap_year(year) {
            [31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
        } else {
            [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
        }
    }

    fn get_days_in_month(month: u8, year: u32) -> u8 {
        DateTime::get_days_in_months(year)[(month - 1) as usize]
    }

    pub fn compare(self, datetime: DateTime) -> bool {
        return self.year == datetime.year 
            && self.month == datetime.month
            && self.day == datetime.day
            && self.hour == datetime.hour
            && self.minute == datetime.minute
            && self.second == datetime.second
    }

    pub fn add_months(&mut self, months: u8) {
        let mut remaining_months = months;
        
        while remaining_months > 0 {
            let months_to_add = remaining_months.min(12 - self.month + 1);
            self.month += months_to_add;
            remaining_months -= months_to_add;

            if remaining_months > 0 {
                self.year += 1;
                self.month = 1;
            }
        }
    }

    fn convert_datetime_to_seconds(&self) -> u64 {
        let mut self_seconds = self.second as u64 + self.minute as u64 * 60 + self.hour as u64 * 3600 + (self.day as u64 - 1) * 86400;
    
        // Add seconds from previous years
        for year in 0..self.year {
            if DateTime::is_leap_year(year) {
                self_seconds += 366 * 86400; // Leap year has 366 days
            } else {
                self_seconds += 365 * 86400; // Non-leap year has 365 days
            }
        }
    
        // Add seconds from previous months in the current year
        for month in 0..(self.month - 1) as usize {
            self_seconds += DateTime::get_days_in_months(self.year)[month] as u64 * 86400;
        }
    
        self_seconds
    }
    

    fn convert_seconds_to_datetime(seconds: u64) -> DateTime {
        let mut remaining_seconds = seconds;
        
        let mut year = 0;
        let mut month = 1;
        let mut day = 1;
        let mut hour = 0;
        let mut minute = 0;
        let mut second = 0;
        
        // Count seconds for years
        while remaining_seconds >= 60 * 60 * 24 * 365 {
            if DateTime::is_leap_year(year) {
                if remaining_seconds >= 60 * 60 * 24 * 366 {
                    remaining_seconds -= 60 * 60 * 24 * 366;
                    year += 1;
                } else {
                    break;
                }
            } else {
                remaining_seconds -= 60 * 60 * 24 * 365;
                year += 1;
            }
        }
        
        // Count seconds for months
        while remaining_seconds >= 60 * 60 * 24 * DateTime::get_days_in_month(month, year) as u64 {
            remaining_seconds -= 60 * 60 * 24 * DateTime::get_days_in_month(month, year) as u64;
            month += 1;
            if month > 12 {
                month = 1;
                year += 1;
            }
        }
        
        // Count seconds for days
        while remaining_seconds >= 60 * 60 * 24 {
            remaining_seconds -= 60 * 60 * 24;
            day += 1;
        }
        
        // Count seconds for hours
        while remaining_seconds >= 60 * 60 {
            remaining_seconds -= 60 * 60;
            hour += 1;
        }
        
        // Count seconds for minutes
        while remaining_seconds >= 60 {
            remaining_seconds -= 60;
            minute += 1;
        }
        
        second = remaining_seconds as u8;
        
        DateTime::new(year, month as u8, day as u8, hour as u8, minute as u8, second)
    }
    
    
    pub fn get_datetime_difference_as_seconds(&self, other: &DateTime) -> u64 {
        let self_seconds = self.convert_datetime_to_seconds();
        let other_seconds = other.convert_datetime_to_seconds();
        other_seconds - self_seconds
    }
}