# Tiny Ruby script to generate some data for a ball
# flying through the air.

# The discrete step is .02 (50 Hz)

InitXVel = 8 # m/s
InitYVel = 5 # m/s
Entropy = 0.05
Prng = Random.new

# adds +/- random noise to value
def noisify_value(value)
  bound = Entropy * value.abs
  noise = Prng.rand(bound)
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

# the actual physics. Assumes X0 = Y0 = 0
def calculate_data(time)
  yDot = InitYVel - 9.8 * time
  x = InitXVel * time
  y = InitYVel * time - 0.5 * 9.8 * time * time

  [x, InitXVel, y, yDot]
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
  # note we are skipping xDot in output, since we don't observe it.
  # we only observe x, y, ydot
  puts "{#{data[0].round(3)},#{data[2].round(3)},#{data[3].round(3)}},"
end
