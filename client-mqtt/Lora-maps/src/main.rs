use std::fs::OpenOptions;
use csv::ReaderBuilder;
use mapplot::google::{style::Color, Circle, GoogleMap};
use std::fs;

#[derive(Debug)]
struct Data {
    gateway_id: String,
    utc_time: String,
    latitude: f64,
    longitude: f64,
    altitude: String,
    rssi: i32,
    snr: String,
    spreading_factor: i8,
}

impl Data {
    fn new(
        gateway_id: String,
        utc_time: String,
        latitude: f64,
        longitude: f64,
        altitude: String,
        rssi: i32,
        snr: String,
        spreading_factor: i8,
    ) -> Self {
        Data {
            gateway_id,
            utc_time,
            latitude,
            longitude,
            altitude,
            rssi,
            snr,
            spreading_factor,
        }
    }
}

fn main() {
    let path = "data.csv";
    let mut options = OpenOptions::new();
    options.read(true);

    let mut csv_reader = match options.open(path) {
        Ok(file) => ReaderBuilder::new().has_headers(true).from_reader(file),
        Err(_) => panic!("Unable to open file"),
    };

    let mut data_vec: Vec<Data> = Vec::new();
    let mut min_rssi = i32::MAX;
    let mut max_rssi = i32::MIN;

    for result in csv_reader.records() {
        match result {
            Ok(record) => {
                let gateway_id = record.get(0).unwrap().to_string();
                let utc_time = record.get(1).unwrap().to_string();
                let latitude = record.get(2).unwrap().parse().unwrap();
                let longitude = record.get(3).unwrap().parse().unwrap();
                let altitude = record.get(4).unwrap().to_string();
                let rssi = record.get(5).unwrap().parse().unwrap();
                let snr = record.get(6).unwrap().to_string();
                let spreading_factor = record.get(7).unwrap().parse().unwrap();

                let data = Data::new(
                    gateway_id,
                    utc_time,
                    latitude,
                    longitude,
                    altitude,
                    rssi,
                    snr,
                    spreading_factor,
                );

                data_vec.push(data);
                min_rssi = min_rssi.min(rssi);
                max_rssi = max_rssi.max(rssi);
            }
            Err(err) => {
                println!("Error reading CSV record: {}", err);
            }
        }
    }

    let mut map_saturation = GoogleMap::new((44.765147, 10.308191), 15, None);
    let mut map = GoogleMap::new((44.765147, 10.308191), 15, None);

    for data in &data_vec {
        let hue = match data.spreading_factor {
            12 => 218, // Blu
            11 => 0,   // Rosso
            10 => 60,  // Giallo
            9 => 280,  // Viola
            7 => 308,
            8 => 158,  // Nero
            _ => 0,    // Valore predefinito (Nero)
        };

        let saturation = map_rssi_to_saturation(data.rssi, min_rssi, max_rssi);
        let color_saturation = Color::HSLA(hue, saturation, 128, 255);
        map_saturation.draw(Circle::new((data.latitude, data.longitude), 20.0).style(color_saturation));
        let color = Color::HSLA(hue, 255, 128, 255);
        map.draw(Circle::new((data.latitude, data.longitude), 20.0).style(color));
    }

    map_saturation.map_type(mapplot::google::MapType::Satellite);
    map.map_type(mapplot::google::MapType::Satellite);
    let html_saturation = map_saturation.to_string();
    let html = map.to_string();
    println!(" max rssi :{}, min rssi: {}",max_rssi,min_rssi);
    println!("{}", html);
    fs::write("map_saturation.html", html_saturation).unwrap();
    fs::write("map.html", html).unwrap();

    std::process::exit(0);
}

fn map_rssi_to_saturation(rssi: i32, min_rssi: i32, max_rssi: i32) -> u8 {
    let saturation_min = 50; // Saturazione minima
    let saturation_max = 255; // Saturazione massima

    let saturation = ((((rssi - min_rssi) as f32 / (max_rssi - min_rssi) as f32) * (saturation_max - saturation_min) as f32) + saturation_min as f32) as u8;

    saturation
}
