# Tiny Ruby script to generate some data for a ball
# flying through the air.

# The discrete step is .02 (50 Hz)

InitX = 8 # m/s
InitY = 5 # m/s
Entropy = 0.05
Prng = Random.new

# adds +/- random noise to value
def noisify_value(value)
  bound = Entropy * value
  noise = Prng.rand(bound.abs)
  noise *= -1 if Prng.rand > 0.5

  value + noise
end

# takes a clean row and makes it noisy
def noisify_row(row)
  [
    noisify_value(row[0]),
    noisify_value(row[1]),
    noisify_value(row[2]),
    noisify_value(row[3])
  ]
end

# the actual physics
def calculate_data(time)
  x = InitX * time
  y = InitY * time - 0.5 * 9.8 * time * time
  xDot = InitX
  yDot = InitY - 9.8 * time * time

  [x, y, xDot, yDot]
end

clean_results = []
noisy_results = []
time = 0
increment = 0.02 # 50Hz


50.times do |i|
  clean_data_row = calculate_data(time)
  clean_results << clean_data_row
  noisy_data_row = noisify_row(clean_data_row)
  noisy_results << noisy_data_row
  time += increment
end

puts "CLEAN DATA:"
clean_results.each do |data|
  puts "#{data[0].round(3)},#{data[1].round(3)},#{data[2].round(3)},#{data[3].round(3)}"
end

puts "-------------------------------------"
puts "NOISY DATA (with #{Entropy} entropy):"
noisy_results.each do |data|
  puts "#{data[0].round(3)},#{data[1].round(3)},#{data[2].round(3)},#{data[3].round(3)}"
end
