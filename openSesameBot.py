import discord

class Client(discord.Client):
    async def on_ready(self):
        print(f'Logged on as {self.user}!');
        
    async def on_message(self, message):
        if message.author == self.user:
            return
        if message.content.startswith('sesame'):
            await message.channel.send(f'Command Recieved: {message.author} executed {message.content}');

    
 
    
    
    
    
    
intents = discord.Intents.default();
intents.message_content = True;

#will fill in after to avoid secret being in repo
botToken = '';


#run bot
client = Client(intents = intents);
client.run(botToken);
